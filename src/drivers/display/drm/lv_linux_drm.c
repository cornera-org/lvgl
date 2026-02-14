/**
 * @file lv_linux_drm.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_linux_drm.h"
#if LV_USE_LINUX_DRM && !LV_LINUX_DRM_USE_EGL

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "../../../stdlib/lv_sprintf.h"
#include "../../../draw/lv_draw_buf.h"
#include "../../../misc/lv_area.h"
#include "../../../misc/lv_area_private.h"
#include "../../../draw/sw/lv_draw_sw_utils.h"

#if LV_USE_LINUX_DRM_GBM_BUFFERS

    #include <gbm.h>
    #include <linux/dma-buf.h>
    #include <sys/ioctl.h>

#endif

/*********************
 *      DEFINES
 *********************/
#if LV_COLOR_DEPTH == 32
    #define DRM_FOURCC DRM_FORMAT_XRGB8888
#elif LV_COLOR_DEPTH == 16
    #define DRM_FOURCC DRM_FORMAT_RGB565
#else
    #error LV_COLOR_DEPTH not supported
#endif

/* Number of DRM framebuffers used for scanout.
 *
 * The original driver used two buffers (double buffering) with DIRECT
 * render mode. We keep the count configurable here; 2 gives classic
 * double buffering, 3 would be triple buffering, etc. The driver always
 * renders into a separate LVGL draw buffer and only ever copies into the
 * currently inactive DRM framebuffer before queuing it for scanout.
 *
 * You can override the default at compile time by defining
 * LV_LINUX_DRM_BUFFER_CNT (for example via CFLAGS) before including this
 * file. */
#ifndef LV_LINUX_DRM_BUFFER_CNT
    #define LV_LINUX_DRM_BUFFER_CNT 2
#endif

#define BUFFER_CNT LV_LINUX_DRM_BUFFER_CNT

/**********************
 *      TYPEDEFS
 **********************/
/* NOTE:
 * This driver has been adapted to work with a separate LVGL draw buffer and
 * software rotation on dumb DRM buffers. The original upstream version
 * configured LVGL to render directly into the DRM framebuffers using
 * LV_DISPLAY_RENDER_MODE_DIRECT and used drm_dmabuf_set_active_buf() +
 * drm_flush_wait() to synchronize with atomic page flips.
 *
 * On this platform that combination triggered crashes when rotation was
 * enabled (LVGL issue 6598), because LVGL's rotation path assumed a
 * separate draw buffer. To avoid that, LVGL now renders into its own
 * 800x480 buffer and this driver rotates that buffer into the 480x800
 * dumb framebuffers in drm_flush().
 */

typedef struct {
    uint32_t handle;
    uint32_t pitch;
    uint32_t offset;
    unsigned long int size;
    uint8_t * map;
    uint32_t fb_handle;
} drm_buffer_t;

typedef struct {
    int fd;
    uint32_t conn_id, enc_id, crtc_id, plane_id, crtc_idx;
    uint32_t width, height;
    uint32_t mmWidth, mmHeight;
    uint32_t fourcc;
    drmModeModeInfo mode;
    uint32_t blob_id;
    drmModeCrtc * saved_crtc;
    drmModeAtomicReq * req;
    drmEventContext drm_event_ctx;
    drmModePlane * plane;
    drmModeCrtc * crtc;
    drmModeConnector * conn;
    uint32_t count_plane_props;
    uint32_t count_crtc_props;
    uint32_t count_conn_props;
    drmModePropertyPtr plane_props[128];
    drmModePropertyPtr crtc_props[128];
    drmModePropertyPtr conn_props[128];
    drm_buffer_t drm_bufs[BUFFER_CNT];

    /* The original driver used LV_DISPLAY_RENDER_MODE_DIRECT and tracked which
     * DRM buffer LVGL was rendering into via act_buf. With the separate draw
     * buffer approach act_buf is no longer used; instead we keep our own
     * framebuffer index and optionally a rotated scratch buffer. */
    drm_buffer_t * act_buf;
    uint32_t fb_index;        /* index of the next DRM buffer to display */
    uint8_t * rotated_buf;    /* optional scratch buffer for rotated output */
    size_t rotated_buf_size;
#if LV_USE_LINUX_DRM_GBM_BUFFERS
    struct gbm_device * gbm_device;
#endif
} drm_dev_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static uint32_t get_plane_property_id(drm_dev_t * drm_dev, const char * name);
static uint32_t get_crtc_property_id(drm_dev_t * drm_dev, const char * name);
static uint32_t get_conn_property_id(drm_dev_t * drm_dev, const char * name);
static void page_flip_handler(int fd, unsigned int sequence, unsigned int tv_sec, unsigned int tv_usec,
                              void * user_data);
static int drm_get_plane_props(drm_dev_t * drm_dev);
static int drm_get_crtc_props(drm_dev_t * drm_dev);
static int drm_get_conn_props(drm_dev_t * drm_dev);
static int drm_add_plane_property(drm_dev_t * drm_dev, const char * name, uint64_t value);
static int drm_add_crtc_property(drm_dev_t * drm_dev, const char * name, uint64_t value);
static int drm_add_conn_property(drm_dev_t * drm_dev, const char * name, uint64_t value);
static int find_plane(drm_dev_t * drm_dev, unsigned int fourcc, uint32_t * plane_id, uint32_t crtc_id,
                      uint32_t crtc_idx);
static int drm_find_connector(drm_dev_t * drm_dev, int64_t connector_id);
static int drm_open(const char * path);
static int drm_setup(drm_dev_t * drm_dev, const char * device_path, int64_t connector_id, unsigned int fourcc);

static uint32_t tick_get_cb(void);

#if !LV_USE_LINUX_DRM_GBM_BUFFERS
    static int drm_allocate_dumb(drm_dev_t * drm_dev, drm_buffer_t * buf);
#elif LV_USE_LINUX_DRM_GBM_BUFFERS
    static int create_gbm_buffer(drm_dev_t * drm_dev, drm_buffer_t * buf);
#endif

static int drm_setup_buffers(drm_dev_t * drm_dev);
static int drm_dmabuf_set_plane(drm_dev_t * drm_dev, drm_buffer_t * buf);
static void drm_flush_wait(lv_display_t * drm_dev);
static void drm_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void drm_dmabuf_set_active_buf(lv_event_t * event);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/
#ifndef DIV_ROUND_UP
    #define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#endif

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_display_t * lv_linux_drm_create(void)
{
    lv_display_t * disp;

    lv_tick_set_cb(tick_get_cb);

    drm_dev_t * drm_dev = lv_malloc_zeroed(sizeof(drm_dev_t));
    LV_ASSERT_MALLOC(drm_dev);
    if(drm_dev == NULL) return NULL;

    drm_dev->fd = -1;

    disp = lv_display_create(800, 480);
    if(disp == NULL) {
        lv_free(drm_dev);
        return NULL;
    }
    lv_display_set_driver_data(disp, drm_dev);

    /* Upstream driver also installed a flush-wait callback and used
     * LV_DISPLAY_RENDER_MODE_DIRECT, letting LVGL render straight into the
     * DRM framebuffers. In this variant LVGL uses a separate draw buffer in
     * FULL mode and drm_flush() is responsible for copying (and rotating)
     * that buffer into the mapped dumb buffers, so only the flush callback
     * is registered here. */
    lv_display_set_flush_cb(disp, drm_flush);

    return disp;
}

/* Called by LVGL when there is something that needs redrawing
 * it sets the active buffer. if GBM buffers are used, it issues a DMA_BUF_SYNC
 * ioctl call to lock the buffer for CPU access, the buffer is unlocked just
 * before the atomic commit */
static void drm_dmabuf_set_active_buf(lv_event_t * event)
{
    /* In the original DIRECT-rendering implementation this callback told LVGL
     * which DRM buffer was currently active so DMA-BUF sync IOCTLs could be
     * issued before CPU access. The reworked driver always renders into a
     * separate LVGL draw buffer (FULL render mode) and only copies into the
     * DRM framebuffers inside drm_flush(), so there is no "active" scanout
     * buffer for LVGL to write to anymore and this hook is effectively
     * unused. It is kept as a no-op to preserve the public API. */
    LV_UNUSED(event);
}

void lv_linux_drm_set_file(lv_display_t * disp, const char * file, int64_t connector_id)
{
    int ret;

    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);

    ret = drm_setup(drm_dev, file, connector_id, DRM_FOURCC);
    if(ret) {
        return;
    }

    ret = drm_setup_buffers(drm_dev);
    if(ret) {
        LV_LOG_ERROR("DRM buffer allocation failed");
        close(drm_dev->fd);
        drm_dev->fd = -1;
        return;
    }

    LV_LOG_INFO("DRM subsystem and buffer mapped successfully");

    int32_t width = drm_dev->mmWidth;

    /* Allocate a separate LVGL draw buffer and use FULL render mode.
     *
     * Original upstream code called lv_display_set_resolution() with the
     * DRM mode's width/height and then lv_display_set_buffers() with the
     * two dumb framebuffers in LV_DISPLAY_RENDER_MODE_DIRECT. That made
     * LVGL render directly into the DRM framebuffers, but also meant that
     * LVGL's rotation and buffer reshaping paths (which assume a distinct
     * draw buffer) could corrupt memory on this platform.
     *
     * Here we keep LVGL's logical resolution (800x480) and give it an
     * internal draw buffer; drm_flush() is responsible for rotating and
     * copying that buffer into the 480x800 DRM framebuffers. */
    int32_t hor_res = lv_display_get_horizontal_resolution(disp);
    int32_t ver_res = lv_display_get_vertical_resolution(disp);
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t stride = lv_draw_buf_width_to_stride(hor_res, cf);
    uint32_t buf_size = stride * ver_res;

    uint8_t *draw_buf = lv_malloc(buf_size);
    LV_ASSERT_MALLOC(draw_buf);
    lv_display_set_buffers(disp, draw_buf, NULL, buf_size, LV_DISPLAY_RENDER_MODE_FULL);

    if(width) {
        lv_display_set_dpi(disp, DIV_ROUND_UP(hor_res * 25400, width * 1000));
    }

    LV_LOG_INFO("Resolution is set to %" LV_PRId32 "x%" LV_PRId32 " at %" LV_PRId32 "dpi",
                hor_res, ver_res, lv_display_get_dpi(disp));
}

void lv_linux_drm_set_mode_cb(lv_display_t * disp, lv_linux_drm_select_mode_cb_t callback)
{
    LV_UNUSED(disp);
    LV_UNUSED(callback);
    LV_LOG_WARN("DRM without EGL support doesn't currently support setting a mode selection callback");
}
/**********************
 *   STATIC FUNCTIONS
 **********************/

static uint32_t get_plane_property_id(drm_dev_t * drm_dev, const char * name)
{
    uint32_t i;

    LV_LOG_TRACE("Find plane property: %s", name);

    for(i = 0; i < drm_dev->count_plane_props; ++i)
        if(!lv_strcmp(drm_dev->plane_props[i]->name, name))
            return drm_dev->plane_props[i]->prop_id;

    LV_LOG_TRACE("Unknown plane property: %s", name);

    return 0;
}

static uint32_t get_crtc_property_id(drm_dev_t * drm_dev, const char * name)
{
    uint32_t i;

    LV_LOG_TRACE("Find crtc property: %s", name);

    for(i = 0; i < drm_dev->count_crtc_props; ++i)
        if(!lv_strcmp(drm_dev->crtc_props[i]->name, name))
            return drm_dev->crtc_props[i]->prop_id;

    LV_LOG_TRACE("Unknown crtc property: %s", name);

    return 0;
}

static uint32_t get_conn_property_id(drm_dev_t * drm_dev, const char * name)
{
    uint32_t i;

    LV_LOG_TRACE("Find conn property: %s", name);

    for(i = 0; i < drm_dev->count_conn_props; ++i)
        if(!lv_strcmp(drm_dev->conn_props[i]->name, name))
            return drm_dev->conn_props[i]->prop_id;

    LV_LOG_TRACE("Unknown conn property: %s", name);

    return 0;
}

static void page_flip_handler(int fd, unsigned int sequence, unsigned int tv_sec, unsigned int tv_usec,
                              void * user_data)
{
    LV_UNUSED(fd);
    LV_UNUSED(sequence);
    LV_UNUSED(tv_sec);
    LV_UNUSED(tv_usec);
    LV_LOG_TRACE("flip");
    drm_dev_t * drm_dev = user_data;
    if(drm_dev->req) {
        drmModeAtomicFree(drm_dev->req);
        drm_dev->req = NULL;
    }
}

static int drm_get_plane_props(drm_dev_t * drm_dev)
{
    uint32_t i;

    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(drm_dev->fd, drm_dev->plane_id,
                                                                  DRM_MODE_OBJECT_PLANE);
    if(!props) {
        LV_LOG_ERROR("drmModeObjectGetProperties failed");
        return -1;
    }
    LV_LOG_TRACE("Found %u plane props", props->count_props);
    drm_dev->count_plane_props = props->count_props;
    for(i = 0; i < props->count_props; i++) {
        drm_dev->plane_props[i] = drmModeGetProperty(drm_dev->fd, props->props[i]);
        LV_LOG_TRACE("Added plane prop %u:%s", drm_dev->plane_props[i]->prop_id, drm_dev->plane_props[i]->name);
    }
    drmModeFreeObjectProperties(props);

    return 0;
}

static int drm_get_crtc_props(drm_dev_t * drm_dev)
{
    uint32_t i;

    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(drm_dev->fd, drm_dev->crtc_id,
                                                                  DRM_MODE_OBJECT_CRTC);
    if(!props) {
        LV_LOG_ERROR("drmModeObjectGetProperties failed");
        return -1;
    }
    LV_LOG_TRACE("Found %u crtc props", props->count_props);
    drm_dev->count_crtc_props = props->count_props;
    for(i = 0; i < props->count_props; i++) {
        drm_dev->crtc_props[i] = drmModeGetProperty(drm_dev->fd, props->props[i]);
        LV_LOG_TRACE("Added crtc prop %u:%s", drm_dev->crtc_props[i]->prop_id, drm_dev->crtc_props[i]->name);
    }
    drmModeFreeObjectProperties(props);

    return 0;
}

static int drm_get_conn_props(drm_dev_t * drm_dev)
{
    uint32_t i;

    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(drm_dev->fd, drm_dev->conn_id,
                                                                  DRM_MODE_OBJECT_CONNECTOR);
    if(!props) {
        LV_LOG_ERROR("drmModeObjectGetProperties failed");
        return -1;
    }
    LV_LOG_TRACE("Found %u connector props", props->count_props);
    drm_dev->count_conn_props = props->count_props;
    for(i = 0; i < props->count_props; i++) {
        drm_dev->conn_props[i] = drmModeGetProperty(drm_dev->fd, props->props[i]);
        LV_LOG_TRACE("Added connector prop %u:%s", drm_dev->conn_props[i]->prop_id, drm_dev->conn_props[i]->name);
    }
    drmModeFreeObjectProperties(props);

    return 0;
}

static int drm_add_plane_property(drm_dev_t * drm_dev, const char * name, uint64_t value)
{
    int ret;
    uint32_t prop_id = get_plane_property_id(drm_dev, name);

    if(!prop_id) {
        LV_LOG_ERROR("Couldn't find plane prop %s", name);
        return -1;
    }

    ret = drmModeAtomicAddProperty(drm_dev->req, drm_dev->plane_id, get_plane_property_id(drm_dev, name), value);
    if(ret < 0) {
        LV_LOG_ERROR("drmModeAtomicAddProperty (%s:%" PRIu64 ") failed: %d", name, value, ret);
        return ret;
    }

    return 0;
}

static int drm_add_crtc_property(drm_dev_t * drm_dev, const char * name, uint64_t value)
{
    int ret;
    uint32_t prop_id = get_crtc_property_id(drm_dev, name);

    if(!prop_id) {
        LV_LOG_ERROR("Couldn't find crtc prop %s", name);
        return -1;
    }

    ret = drmModeAtomicAddProperty(drm_dev->req, drm_dev->crtc_id, get_crtc_property_id(drm_dev, name), value);
    if(ret < 0) {
        LV_LOG_ERROR("drmModeAtomicAddProperty (%s:%" PRIu64 ") failed: %d", name, value, ret);
        return ret;
    }

    return 0;
}

static int drm_add_conn_property(drm_dev_t * drm_dev, const char * name, uint64_t value)
{
    int ret;
    uint32_t prop_id = get_conn_property_id(drm_dev, name);

    if(!prop_id) {
        LV_LOG_ERROR("Couldn't find conn prop %s", name);
        return -1;
    }

    ret = drmModeAtomicAddProperty(drm_dev->req, drm_dev->conn_id, get_conn_property_id(drm_dev, name), value);
    if(ret < 0) {
        LV_LOG_ERROR("drmModeAtomicAddProperty (%s:%" PRIu64 ") failed: %d", name, value, ret);
        return ret;
    }

    return 0;
}

static int drm_dmabuf_set_plane(drm_dev_t * drm_dev, drm_buffer_t * buf)
{
    int ret;
    static int first = 1;
    uint32_t flags = DRM_MODE_PAGE_FLIP_EVENT | DRM_MODE_ATOMIC_NONBLOCK;

#if LV_USE_LINUX_DRM_GBM_BUFFERS

    struct dma_buf_sync sync_req;

    sync_req.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
    if(ioctl(buf->handle, DMA_BUF_IOCTL_SYNC, &sync_req) != 0) {
        LV_LOG_ERROR("Failed to end DMA-BUF R/W SYNC");
    }

#endif

    drm_dev->req = drmModeAtomicAlloc();

    /* On first Atomic commit, do a modeset */
    if(first) {
        drm_add_conn_property(drm_dev, "CRTC_ID", drm_dev->crtc_id);

        drm_add_crtc_property(drm_dev, "MODE_ID", drm_dev->blob_id);
        drm_add_crtc_property(drm_dev, "ACTIVE", 1);

        flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;

        first = 0;
    }

    drm_add_plane_property(drm_dev, "FB_ID", buf->fb_handle);
    drm_add_plane_property(drm_dev, "CRTC_ID", drm_dev->crtc_id);
    drm_add_plane_property(drm_dev, "SRC_X", 0);
    drm_add_plane_property(drm_dev, "SRC_Y", 0);
    drm_add_plane_property(drm_dev, "SRC_W", drm_dev->width << 16);
    drm_add_plane_property(drm_dev, "SRC_H", drm_dev->height << 16);
    drm_add_plane_property(drm_dev, "CRTC_X", 0);
    drm_add_plane_property(drm_dev, "CRTC_Y", 0);
    drm_add_plane_property(drm_dev, "CRTC_W", drm_dev->width);
    drm_add_plane_property(drm_dev, "CRTC_H", drm_dev->height);

    ret = drmModeAtomicCommit(drm_dev->fd, drm_dev->req, flags, drm_dev);
    if(ret) {
        LV_LOG_ERROR("drmModeAtomicCommit failed: %s (%d)", strerror(errno), errno);
        drmModeAtomicFree(drm_dev->req);
        drm_dev->req = NULL;
        return ret;
    }

    return 0;
}

static int find_plane(drm_dev_t * drm_dev, unsigned int fourcc, uint32_t * plane_id, uint32_t crtc_id,
                      uint32_t crtc_idx)
{
    LV_UNUSED(crtc_id);
    drmModePlaneResPtr planes;
    drmModePlanePtr plane;
    unsigned int i;
    unsigned int j;
    int ret = 0;
    unsigned int format = fourcc;

    planes = drmModeGetPlaneResources(drm_dev->fd);
    if(!planes) {
        LV_LOG_ERROR("drmModeGetPlaneResources failed");
        return -1;
    }

    LV_LOG_TRACE("drm: found planes %u", planes->count_planes);

    for(i = 0; i < planes->count_planes; ++i) {
        plane = drmModeGetPlane(drm_dev->fd, planes->planes[i]);
        if(!plane) {
            LV_LOG_ERROR("drmModeGetPlane failed: %s", strerror(errno));
            break;
        }

        if(!(plane->possible_crtcs & (1 << crtc_idx))) {
            drmModeFreePlane(plane);
            continue;
        }

        for(j = 0; j < plane->count_formats; ++j) {
            if(plane->formats[j] == format)
                break;
        }

        if(j == plane->count_formats) {
            drmModeFreePlane(plane);
            continue;
        }

        *plane_id = plane->plane_id;
        drmModeFreePlane(plane);

        LV_LOG_TRACE("found plane %d", *plane_id);

        break;
    }

    if(i == planes->count_planes)
        ret = -1;

    drmModeFreePlaneResources(planes);

    return ret;
}

static int drm_find_connector(drm_dev_t * drm_dev, int64_t connector_id)
{
    drmModeConnector * conn = NULL;
    drmModeEncoder * enc = NULL;
    drmModeRes * res;
    int i;

    if((res = drmModeGetResources(drm_dev->fd)) == NULL) {
        LV_LOG_ERROR("drmModeGetResources() failed");
        return -1;
    }

    if(res->count_crtcs <= 0) {
        LV_LOG_ERROR("no Crtcs");
        goto free_res;
    }

    /* find all available connectors */
    for(i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(drm_dev->fd, res->connectors[i]);
        if(!conn)
            continue;

        if(connector_id >= 0 && conn->connector_id != connector_id) {
            drmModeFreeConnector(conn);
            continue;
        }

        if(conn->connection == DRM_MODE_CONNECTED) {
            LV_LOG_TRACE("drm: connector %d: connected", conn->connector_id);
        }
        else if(conn->connection == DRM_MODE_DISCONNECTED) {
            LV_LOG_TRACE("drm: connector %d: disconnected", conn->connector_id);
        }
        else if(conn->connection == DRM_MODE_UNKNOWNCONNECTION) {
            LV_LOG_TRACE("drm: connector %d: unknownconnection", conn->connector_id);
        }
        else {
            LV_LOG_TRACE("drm: connector %d: unknown", conn->connector_id);
        }

        if(conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0)
            break;

        drmModeFreeConnector(conn);
        conn = NULL;
    };

    if(!conn) {
        LV_LOG_ERROR("suitable connector not found");
        goto free_res;
    }

    drm_dev->conn_id = conn->connector_id;
    LV_LOG_TRACE("conn_id: %d", drm_dev->conn_id);
    drm_dev->mmWidth = conn->mmWidth;
    drm_dev->mmHeight = conn->mmHeight;

    lv_memcpy(&drm_dev->mode, &conn->modes[0], sizeof(drmModeModeInfo));

    if(drmModeCreatePropertyBlob(drm_dev->fd, &drm_dev->mode, sizeof(drm_dev->mode),
                                 &drm_dev->blob_id)) {
        LV_LOG_ERROR("error creating mode blob");
        goto free_res;
    }

    drm_dev->width = conn->modes[0].hdisplay;
    drm_dev->height = conn->modes[0].vdisplay;

    for(i = 0 ; i < res->count_encoders; i++) {
        enc = drmModeGetEncoder(drm_dev->fd, res->encoders[i]);
        if(!enc)
            continue;

        LV_LOG_TRACE("enc%d enc_id %d conn enc_id %d", i, enc->encoder_id, conn->encoder_id);

        if(enc->encoder_id == conn->encoder_id)
            break;

        drmModeFreeEncoder(enc);
        enc = NULL;
    }

    if(enc) {
        drm_dev->enc_id = enc->encoder_id;
        LV_LOG_TRACE("enc_id: %d", drm_dev->enc_id);
        drm_dev->crtc_id = enc->crtc_id;
        LV_LOG_TRACE("crtc_id: %d", drm_dev->crtc_id);
        drmModeFreeEncoder(enc);
    }
    else {
        /* Encoder hasn't been associated yet, look it up */
        for(i = 0; i < conn->count_encoders; i++) {
            int crtc, crtc_id = -1;

            enc = drmModeGetEncoder(drm_dev->fd, conn->encoders[i]);
            if(!enc)
                continue;

            for(crtc = 0 ; crtc < res->count_crtcs; crtc++) {
                uint32_t crtc_mask = 1 << crtc;

                crtc_id = res->crtcs[crtc];

                LV_LOG_TRACE("enc_id %d crtc%d id %d mask %x possible %x", enc->encoder_id, crtc, crtc_id, crtc_mask,
                             enc->possible_crtcs);

                if(enc->possible_crtcs & crtc_mask)
                    break;
            }

            if(crtc_id > 0) {
                drm_dev->enc_id = enc->encoder_id;
                LV_LOG_TRACE("enc_id: %d", drm_dev->enc_id);
                drm_dev->crtc_id = crtc_id;
                LV_LOG_TRACE("crtc_id: %d", drm_dev->crtc_id);
                break;
            }

            drmModeFreeEncoder(enc);
            enc = NULL;
        }

        if(!enc) {
            LV_LOG_ERROR("suitable encoder not found");
            goto free_res;
        }

        drmModeFreeEncoder(enc);
    }

    drm_dev->crtc_idx = UINT32_MAX;

    for(i = 0; i < res->count_crtcs; ++i) {
        if(drm_dev->crtc_id == res->crtcs[i]) {
            drm_dev->crtc_idx = i;
            break;
        }
    }

    if(drm_dev->crtc_idx == UINT32_MAX) {
        LV_LOG_ERROR("drm: CRTC not found");
        goto free_res;
    }

    LV_LOG_TRACE("crtc_idx: %d", drm_dev->crtc_idx);

    return 0;

free_res:
    drmModeFreeResources(res);

    return -1;
}

static int drm_open(const char * path)
{
    int fd, flags;
    uint64_t has_dumb;
    int ret;

    fd = open(path, O_RDWR);
    if(fd < 0) {
        LV_LOG_ERROR("cannot open \"%s\"", path);
        return -1;
    }

    /* set FD_CLOEXEC flag */
    if((flags = fcntl(fd, F_GETFD)) < 0 ||
       fcntl(fd, F_SETFD, flags | FD_CLOEXEC) < 0) {
        LV_LOG_ERROR("fcntl FD_CLOEXEC failed");
        goto err;
    }

    /* check capability */
    ret = drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &has_dumb);
    if(ret < 0 || has_dumb == 0) {
        LV_LOG_ERROR("drmGetCap DRM_CAP_DUMB_BUFFER failed or \"%s\" doesn't have dumb "
                     "buffer", path);
        goto err;
    }

    return fd;
err:
    close(fd);
    return -1;
}

static int drm_setup(drm_dev_t * drm_dev, const char * device_path, int64_t connector_id, unsigned int fourcc)
{
    int ret;

    drm_dev->fd = drm_open(device_path);
    if(drm_dev->fd < 0)
        return -1;

    ret = drmSetClientCap(drm_dev->fd, DRM_CLIENT_CAP_ATOMIC, 1);
    if(ret) {
        LV_LOG_ERROR("No atomic modesetting support: %s", strerror(errno));
        goto err;
    }

    ret = drm_find_connector(drm_dev, connector_id);
    if(ret) {
        LV_LOG_ERROR("available drm devices not found");
        goto err;
    }

    ret = find_plane(drm_dev, fourcc, &drm_dev->plane_id, drm_dev->crtc_id, drm_dev->crtc_idx);
    if(ret) {
        LV_LOG_ERROR("Cannot find plane");
        goto err;
    }

    drm_dev->plane = drmModeGetPlane(drm_dev->fd, drm_dev->plane_id);
    if(!drm_dev->plane) {
        LV_LOG_ERROR("Cannot get plane");
        goto err;
    }

    drm_dev->crtc = drmModeGetCrtc(drm_dev->fd, drm_dev->crtc_id);
    if(!drm_dev->crtc) {
        LV_LOG_ERROR("Cannot get crtc");
        goto err;
    }

    drm_dev->conn = drmModeGetConnector(drm_dev->fd, drm_dev->conn_id);
    if(!drm_dev->conn) {
        LV_LOG_ERROR("Cannot get connector");
        goto err;
    }

    ret = drm_get_plane_props(drm_dev);
    if(ret) {
        LV_LOG_ERROR("Cannot get plane props");
        goto err;
    }

    ret = drm_get_crtc_props(drm_dev);
    if(ret) {
        LV_LOG_ERROR("Cannot get crtc props");
        goto err;
    }

    ret = drm_get_conn_props(drm_dev);
    if(ret) {
        LV_LOG_ERROR("Cannot get connector props");
        goto err;
    }

    drm_dev->drm_event_ctx.version = DRM_EVENT_CONTEXT_VERSION;
    drm_dev->drm_event_ctx.page_flip_handler = page_flip_handler;
    drm_dev->fourcc = fourcc;

    LV_LOG_INFO("drm: Found plane_id: %u connector_id: %d crtc_id: %d",
                drm_dev->plane_id, drm_dev->conn_id, drm_dev->crtc_id);

    LV_LOG_INFO("drm: %dx%d (%dmm X% dmm) pixel format %c%c%c%c",
                drm_dev->width, drm_dev->height, drm_dev->mmWidth, drm_dev->mmHeight,
                (fourcc >> 0) & 0xff, (fourcc >> 8) & 0xff, (fourcc >> 16) & 0xff, (fourcc >> 24) & 0xff);


#if LV_USE_LINUX_DRM_GBM_BUFFERS

    /* Create GBM device and buffer */
    drm_dev->gbm_device = gbm_create_device(drm_dev->fd);

    if(drm_dev->gbm_device == NULL) {
        LV_LOG_ERROR("Failed to create GBM device");
        goto err;
    }

    LV_LOG_INFO("GBM device backend: %s", gbm_device_get_backend_name(drm_dev->gbm_device));
#endif

    return 0;

err:
    close(drm_dev->fd);
    return -1;
}

#if !LV_USE_LINUX_DRM_GBM_BUFFERS
static int drm_allocate_dumb(drm_dev_t * drm_dev, drm_buffer_t * buf)
{
    struct drm_mode_create_dumb creq;
    struct drm_mode_map_dumb mreq;
    uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};
    int ret;

    /* create dumb buffer */
    lv_memzero(&creq, sizeof(creq));
    creq.width = drm_dev->width;
    creq.height = drm_dev->height;
    creq.bpp = LV_COLOR_DEPTH;
    ret = drmIoctl(drm_dev->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);
    if(ret < 0) {
        LV_LOG_ERROR("DRM_IOCTL_MODE_CREATE_DUMB fail");
        return -1;
    }

    buf->handle = creq.handle;
    buf->pitch = creq.pitch;
    buf->size = creq.size;

    /* prepare buffer for memory mapping */
    lv_memzero(&mreq, sizeof(mreq));
    mreq.handle = creq.handle;
    ret = drmIoctl(drm_dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);
    if(ret) {
        LV_LOG_ERROR("DRM_IOCTL_MODE_MAP_DUMB fail");
        return -1;
    }

    buf->offset = mreq.offset;
    LV_LOG_INFO("size %lu pitch %u offset %u", buf->size, buf->pitch, buf->offset);

    /* perform actual memory mapping */
    buf->map = mmap(0, creq.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_dev->fd, mreq.offset);
    if(buf->map == MAP_FAILED) {
        LV_LOG_ERROR("mmap fail");
        return -1;
    }

    /* clear the framebuffer to 0 (= full transparency in ARGB8888) */
    lv_memzero(buf->map, creq.size);

    /* create framebuffer object for the dumb-buffer */
    handles[0] = creq.handle;
    pitches[0] = creq.pitch;
    offsets[0] = 0;
    ret = drmModeAddFB2(drm_dev->fd, drm_dev->width, drm_dev->height, drm_dev->fourcc,
                        handles, pitches, offsets, &buf->fb_handle, 0);
    if(ret) {
        LV_LOG_ERROR("drmModeAddFB fail");
        return -1;
    }

    return 0;
}
#endif /*!LV_USE_LINUX_DRM_GBM_BUFFERS*/

#if LV_USE_LINUX_DRM_GBM_BUFFERS

static int create_gbm_buffer(drm_dev_t * drm_dev, drm_buffer_t * buf)
{

    struct gbm_bo * gbm_bo;
    int prime_fd;
    uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};
    uint32_t n_planes;
    int res;

    /* gbm_bo_format does not define anything other than ARGB8888 or XRGB8888 */
    if(LV_COLOR_DEPTH != 32) {
        LV_LOG_ERROR("Unsupported color format");
        return -1;
    }

    /* Create a linear GBM buffer object - best practice when modifiers are not used */
    if(!(gbm_bo = gbm_bo_create(drm_dev->gbm_device,
                                drm_dev->width, drm_dev->height, GBM_BO_FORMAT_XRGB8888,
                                GBM_BO_USE_SCANOUT | GBM_BO_USE_LINEAR))) {

        LV_LOG_ERROR("Unable to create gbm buffer object");
        return -1;
    }

    /* Currently only, one plane per dma-buf/prime fd is supported - but some GPUs feature
     * several planes (multiple fds or sometimes a single fd for multiple planes).
     * current implementation is kept simple for now */

    n_planes = gbm_bo_get_plane_count(gbm_bo);

    if(n_planes != 1) {
        LV_LOG_ERROR("The current implementation only supports a single plane per fd");
        return -1;
    }

    uint32_t h = gbm_bo_get_height(gbm_bo);
    pitches[0] = buf->pitch = gbm_bo_get_stride_for_plane(gbm_bo, 0);
    offsets[0] = buf->offset = gbm_bo_get_offset(gbm_bo, 0);
    buf->size = h * buf->pitch;

    LV_LOG_INFO("Created GBM BO of size: %lu pitch: %u offset: %u",
                buf->size, buf->pitch, buf->offset);

    prime_fd = gbm_bo_get_fd_for_plane(gbm_bo, 0);

    if(prime_fd < 0) {
        LV_LOG_ERROR("Failed to get prime fd for plane 0");
        return -1;

    }

    buf->map = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, prime_fd, 0);

    if(buf->map == MAP_FAILED) {
        LV_LOG_ERROR("Failed to mmap dma-buf fd.");
        return -1;
    }

    /* Used to perform DMA_BUF_SYNC ioctl calls during the rendering cycle */
    buf->handle = prime_fd;

    /* Convert prime fd to a libdrm buffer handle */
    drmPrimeFDToHandle(drm_dev->fd, buf->handle, &handles[0]);

    /* create libdrm framebuffer */
    res = drmModeAddFB2(drm_dev->fd, drm_dev->width, drm_dev->height, drm_dev->fourcc,
                        handles, pitches, offsets, &buf->fb_handle, 0);

    if(res) {
        LV_LOG_ERROR("drmModeAddFB2 failed");
        return -1;
    }

    if(drmCloseBufferHandle(drm_dev->fd, handles[0]) != 0) {
        LV_LOG_ERROR("drmCloseBufferHandle failed");
        return -1;
    }

    return 0;

}

#endif /* LV_USE_LINUX_DRM_GBM_BUFFERS */

static int drm_setup_buffers(drm_dev_t * drm_dev)
{
    int ret;

#if LV_USE_LINUX_DRM_GBM_BUFFERS
    for(int i = 0; i < BUFFER_CNT; i++) {
        ret = create_gbm_buffer(drm_dev, &drm_dev->drm_bufs[i]);
        if(ret < 0) {
            return ret;
        }
    }
#else

    /* Use dumb buffers */
    for(int i = 0; i < BUFFER_CNT; i++) {
        ret = drm_allocate_dumb(drm_dev, &drm_dev->drm_bufs[i]);
        if(ret)
            return ret;
    }

    drm_dev->fb_index = 0;
    drm_dev->rotated_buf = NULL;
    drm_dev->rotated_buf_size = 0;

#endif

    return 0;
}

static void drm_flush_wait(lv_display_t * disp)
{
    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);

    struct pollfd pfd;
    pfd.fd = drm_dev->fd;
    pfd.events = POLLIN;

    while(drm_dev->req) {
        int ret;
        do {
            ret = poll(&pfd, 1, -1);
        } while(ret == -1 && errno == EINTR);

        if(ret > 0)
            drmHandleEvent(drm_dev->fd, &drm_dev->drm_event_ctx);
        else {
            LV_LOG_ERROR("poll failed: %s", strerror(errno));
            return;
        }
    }
}

static void drm_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    LV_UNUSED(area);

    drm_dev_t * drm_dev = lv_display_get_driver_data(disp);
    if(!drm_dev) {
        lv_display_flush_ready(disp);
        return;
    }

    /* Atomic commits are queued with NONBLOCK and retired from page-flip
     * events. Wait for any in-flight commit to finish before queuing the next
     * one, otherwise requests/events can pile up and eventually fail with
     * ENOMEM on some DRM drivers. */
    drm_flush_wait(disp);

    const lv_color_format_t cf = lv_display_get_color_format(disp);
    const uint32_t px_size = lv_color_format_get_size(cf);

    int32_t src_w = lv_display_get_horizontal_resolution(disp);
    int32_t src_h = lv_display_get_vertical_resolution(disp);
    uint32_t src_stride = lv_draw_buf_width_to_stride(src_w, cf);

    /* Select back buffer and rotate the whole LVGL buffer into it.
     *
     * The LVGL UI is designed for 800x480, while the panel is mounted
     * vertically as 480x800. Instead of asking LVGL to rotate its own
     * draw buffer (which was not safe with DIRECT mode), we always render
     * unrotated into the LVGL draw buffer and apply a fixed 270-degree
     * rotation here when copying into the DRM framebuffers. */
    drm_buffer_t * buf = &drm_dev->drm_bufs[drm_dev->fb_index];
    drm_dev->fb_index = (drm_dev->fb_index + 1) % BUFFER_CNT;

    /* Panel is mounted vertically (480x800), UI is 800x480: rotate 270 degrees. */
    lv_draw_sw_rotate(px_map,
                      buf->map,
                      src_w,
                      src_h,
                      (int32_t)src_stride,
                      (int32_t)buf->pitch,
                      LV_DISPLAY_ROTATION_270,
                      cf);

    if(drm_dmabuf_set_plane(drm_dev, buf)) {
        LV_LOG_ERROR("Flush fail");
    }

    lv_display_flush_ready(disp);

}

static uint32_t tick_get_cb(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t time_ms = t.tv_sec * 1000 + (t.tv_nsec / 1000000);
    return time_ms;
}

#endif /*LV_USE_LINUX_DRM && !LV_LINUX_DRM_USE_EGL*/
