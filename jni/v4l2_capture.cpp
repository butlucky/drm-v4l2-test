/*
 * V4L2 video capture
 * AUTHER : hewenwen
 * DATA   : 2023-03-20
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h> /* getopt_long() */
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <dlfcn.h>
#include <signal.h>
#include <dirent.h>
#include <time.h>
#include <poll.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <pthread.h>
#include <sys/system_properties.h>
#include "utils/log_utils.h"
#include "v4l2/videodev2.h"
#ifdef V4L2_BUILD_DAEMO
#include "video_render/render/VideoRendererManager.h"
#include "common/video_frame/VideoFrame.h"
static std::shared_ptr<byteview::VideoRenderer> v4l2_videoRenderer;
#endif

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define FMT_NUM_PLANES 1
#define BUFFER_COUNT 4
#define HASH_TABLE_NUM 128
#define CAPTURE_RAW_PATH "/data"
#define DEFAULT_CAPTURE_RAW_PATH "/data/capture_image"
#define CAPTURE_CNT_FILENAME ".capture_cnt"

enum Cam_Ctrl {
    /* view mode settings */
    CAM_MODE_DEFAULT,
    CAM_MODE_SOFT,
    CAM_MODE_STANDARD,
    CAM_MODE_CLEAR,
    CAM_MODE_BRIGHT,
    CAM_MODE_COMPUTER,
    CAM_MODE_CLEAR_LED,
    CAM_MODE_FACIAL_FEATURES,

    /* ptz control */
    CAM_CTRL_PAN_LEFT,
    CAM_CTRL_PAN_RIGHT,
    CAM_CTRL_TILT_UP,
    CAM_CTRL_TILT_DOWN,
    CAM_CTRL_ZOOM_IN,
    CAM_CTRL_ZOOM_OUT,
};

typedef struct node {
    int key;
    int min;
    int max;
    int range;
    struct node *next;
} vidctl_node;

typedef struct v4l2_context {
    char out_file[255];
    char dev_name[255];
    int width;
    int height;
    int stride_x;
    int stride_y;
    int srcfps;
    int dstfps;
    int format;
    int memtype;
    int vop;
    int fd;
    int drm_fd;
    enum v4l2_buf_type buf_type;
    struct buffer *buffers;
    unsigned int n_buffers;
    int frame_count;
    FILE *fp;
    int writeFile;
    int writeFileSync;
    int hdrmode;
    int limit_range;
    int outputCnt;
    int skipCnt;
    struct video_fmts *v_infos;
    unsigned int nv_infos;
    int pipe;
    vidctl_node *vidctl[HASH_TABLE_NUM];

    char yuv_dir_path[64];
    int _is_yuv_dir_exist;
    int capture_yuv_num;
    int is_capture_yuv;
    unsigned int log_interval;
} v4l2_context_t;

struct buffer {
    void *start;
    long length;
    int export_fd;
    int handle;
    int sequence;
};

struct video_fmts {
	int pixelformat;
	int fmt_type;
	int fmt_cnts;
	void *fmt_info;
};

static int silent;
static v4l2_context_t *g_main_ctx = NULL;
static const char *dev_drm = "/dev/dri/card0";
static const char *p_fifo = "/mnt/.fifobyted";
pthread_t camera_control;
void *camera_thread(void *arg);

#ifdef V4L2_BUILD_DAEMO
static byteview::FrameType frmtype = byteview::FrameType::kNv12_OESFD;
static void init_render(v4l2_context_t *ctx);
static void deinit_render(v4l2_context_t *ctx);
#define DBG(...) do { if(!silent) { printf("[%s-%d]--> ",__func__,__LINE__); printf(__VA_ARGS__);} } while(0)
#define ERR(...) do { printf("[%s-%d]--> ",__func__,__LINE__); printf(__VA_ARGS__); } while (0)
#else
#define DBG BYTE_LOGD
#define ERR BYTE_LOGE
#endif

static void errno_exit(v4l2_context_t *ctx, const char *s)
{
    ERR("%s: %s error %d, %s\n", ctx->dev_name, s, errno, strerror(errno));
    if(!ctx->vop)
        exit(-1);
}

static int xioctl(int fh, int request, void *arg)
{
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

static int drm_open()
{
    int ret = 0;

    ret = open(dev_drm, O_RDWR | O_CLOEXEC);
    if (ret < 0) {
        ERR("open %s failed!\n", dev_drm);
        ret = -1;
    }

    return ret;
}

static int drm_close(int fd)
{
    int ret = 0;

    ret = close(fd);
    if (ret < 0) {
        ERR("close %s failed!\n", dev_drm);
        ret = -1;
    }

    return ret;
}

static int drm_alloc(int drm_fd, size_t width, size_t height, struct buffer *buffinfo, int fmt)
{
    int ret;
    struct drm_mode_create_dumb dmcb;

    CLEAR(dmcb);
    dmcb.bpp  = 8;
    switch(fmt){
    default:
        DBG("fix me!! using default nv12 format!\n");
    case V4L2_PIX_FMT_NV12:
    case V4L2_PIX_FMT_NV21:
        dmcb.width  = width;
        dmcb.height = height*3/2;
        break;
    case V4L2_PIX_FMT_NV16:
    case V4L2_PIX_FMT_NV61:
        dmcb.width  = width;
        dmcb.height = height*2;
        break;
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YVYU:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_VYUY:
        dmcb.width  = width*2;
        dmcb.height = height;
        break;
    }
    /* aligned with 16 bytes */
    dmcb.height = (dmcb.height+15)&~15;

    ret = xioctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &dmcb);
    if (ret < 0){
        ERR("drm create dumb error\n");
        return ret;
    }

    buffinfo->handle = dmcb.handle;
    buffinfo->length = dmcb.size;
    DBG(" handle %d pitch %d size %lld\n",
        dmcb.handle, dmcb.pitch, dmcb.size);

    return ret;
}

static int drm_handle_to_fd(int drm_fd, struct buffer *buffinfo)
{
    int ret;
    struct drm_prime_handle dph;

    CLEAR(dph);
    dph.handle = buffinfo->handle;
    dph.fd = -1;

    ret = xioctl(drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &dph);
    if (ret < 0){
        ERR("failed \n");
        ret = -1;
    }

    buffinfo->export_fd = dph.fd;

    DBG(" handle %d get fd %d\n", dph.handle, dph.fd);

    return ret;
}

static int drm_map(int drm_fd, struct buffer *buffinfo)
{
    int ret = 0;
    struct drm_mode_map_dumb map;

    CLEAR(map);
    map.handle = buffinfo->handle;
    ret = xioctl(drm_fd,DRM_IOCTL_MODE_MAP_DUMB,&map);
    if(ret < 0){
        ERR("drm map dumb error\n");
        return ret;
    }

    buffinfo->start =  mmap(NULL /* start anywhere */,
            buffinfo->length,PROT_READ | PROT_WRITE /* required */,
            MAP_SHARED /* recommended */, drm_fd, map.offset);

    if (MAP_FAILED == buffinfo->start){
        ret = -1;
        ERR("can not get valid memmap space \n");
    }
    DBG(" handle %d offset 0x%llx map-vliraddr 0x%lx\n",
        map.handle, map.offset, (long)buffinfo->start);

    return ret;
}

static int drm_free(int drm_fd, struct buffer *buffinfo)
{
    struct drm_mode_destroy_dumb data = {
        .handle = (unsigned int)buffinfo->handle,
    };

    return xioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &data);
}

static void drm_init_mmap(v4l2_context_t *ctx)
{
    struct v4l2_requestbuffers req;
    struct buffer *tmp_buffers = NULL;

    CLEAR(req);
    req.count = BUFFER_COUNT;
    req.type = ctx->buf_type;
    req.memory = V4L2_MEMORY_DMABUF;

    if (-1 == xioctl(ctx->fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            ERR("%s does not support "
                "dma bufs \n", ctx->dev_name);
        } else {
            errno_exit(ctx, "VIDIOC_REQBUFS");
        }
    }
    if (req.count < 2) {
        ERR("%s: Insufficient buffer memory on %s\n", ctx->dev_name,
            ctx->dev_name);
    }

    tmp_buffers = (struct buffer*)calloc(req.count, sizeof(struct buffer));
    if (!tmp_buffers) {
        ERR("%s: Out of memory\n", ctx->dev_name);
    }
    ctx->buffers = tmp_buffers;
    for (ctx->n_buffers = 0; ctx->n_buffers < req.count; ++ctx->n_buffers) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[FMT_NUM_PLANES];

        CLEAR(buf);
        CLEAR(planes);
        buf.index = ctx->n_buffers;
        buf.type = ctx->buf_type;
        buf.memory = V4L2_MEMORY_DMABUF;
        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
            buf.m.planes = planes;
            buf.length = FMT_NUM_PLANES;
        }

        if (-1 == xioctl(ctx->fd, VIDIOC_QUERYBUF, &buf))
            errno_exit(ctx, "VIDIOC_QUERYBUF");

        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
            DBG("v4l2_buffer [type: %d, bytesused %d], "
                "v4l2_plane [bytesused %d, length %d, mem_offset %d ,data_offset %d]\n",
                buf.type, buf.bytesused, buf.m.planes[0].bytesused, buf.m.planes[0].length,
                buf.m.planes[0].m.mem_offset, buf.m.planes[0].data_offset);
        }else{
            DBG("v4l2_buffer [type: %d, bytesused %d, mem_offset %d, length %d] \n ",
                buf.type, buf.bytesused,buf.m.offset, buf.length);
        }

        DBG("prepare dma_bufs, index [%d]\n",ctx->n_buffers);
        drm_alloc(ctx->drm_fd, ctx->width, ctx->height,
                &tmp_buffers[ctx->n_buffers], ctx->format);
        drm_handle_to_fd(ctx->drm_fd, &tmp_buffers[ctx->n_buffers]);
        drm_map(ctx->drm_fd, &tmp_buffers[ctx->n_buffers]);
    }
}

static int get_value_from_file(const char* path, int* value, int* frameId)
{
    const char *delim = " ";
    char buffer[16] = {0};
    int fp;

    fp = open(path, O_RDONLY | O_SYNC | O_CLOEXEC);
    if (fp) {
        if (read(fp, buffer, sizeof(buffer)) > 0) {
            char *p = NULL;

            p = strtok(buffer, delim);
            if (p != NULL) {
                *value = atoi(p);
                p = strtok(NULL, delim);
                if (p != NULL)
                    *frameId = atoi(p);
            }
        }
        close(fp);
        return 1;
    }

    return 0;
}

static int write_yuv_to_file(const void *p,
                             int size, int sequence, v4l2_context_t *ctx)
{
    char file_name[64] = {0};

    snprintf(file_name, sizeof(file_name),
             "%s/frame%d.yuv",
             ctx->yuv_dir_path,
             sequence);
    ctx->fp = fopen(file_name, "wb+");
    if (ctx->fp == NULL) {
        ERR("fopen yuv file %s failed!\n", file_name);
        return -1;
    }

    fwrite(p, size, 1, ctx->fp);
    fflush(ctx->fp);

    if (ctx->fp) {
        fclose(ctx->fp);
        ctx->fp = NULL;
    }

    for (int i = 0; i < ctx->capture_yuv_num; i++)
        DBG("<");
    DBG("\n");

    return 0;
}

static int creat_yuv_dir(const char* path, v4l2_context_t *ctx)
{
    time_t now;
    struct tm* timenow;

    if (!path)
        return -1;

    time(&now);
    timenow = localtime(&now);
    snprintf(ctx->yuv_dir_path, sizeof(ctx->yuv_dir_path),
             "%s/yuv_%04d-%02d-%02d_%02d-%02d-%02d",
             path,
             timenow->tm_year + 1900,
             timenow->tm_mon + 1,
             timenow->tm_mday,
             timenow->tm_hour,
             timenow->tm_min,
             timenow->tm_sec);

    DBG("mkdir %s for capturing yuv!\n", ctx->yuv_dir_path);

    if(mkdir(ctx->yuv_dir_path, 0755) < 0) {
        DBG("mkdir %s error!!!\n", ctx->yuv_dir_path);
        return -1;
    }

    ctx->_is_yuv_dir_exist = 1;

    return 0;
}

static void process_image(const void *p, int sequence, int size, v4l2_context_t *ctx)
{
    if (ctx->fp && sequence >= ctx->skipCnt && ctx->outputCnt-- > 0) {
        if(ctx->log_interval > 0x80000000)
            DBG("frame : <%d> \n",sequence+1);
        fwrite(p, size, 1, ctx->fp);
        fflush(ctx->fp);
    } else if (ctx->writeFileSync) {
        int ret = 0;
        if (!ctx->is_capture_yuv) {
            char file_name[32] = {0};
            int rawFrameId = 0;

            snprintf(file_name, sizeof(file_name), "%s/%s",
                     CAPTURE_RAW_PATH, CAPTURE_CNT_FILENAME);
            get_value_from_file(file_name, &ctx->capture_yuv_num, &rawFrameId);

            /*
             * DBG("%s: rawFrameId: %d, sequence: %d\n", __FUNCTION__,
             *        rawFrameId, sequence);
             */

            sequence += 1;
            if (ctx->capture_yuv_num > 0 && \
                    ((sequence >= rawFrameId && rawFrameId > 0) || sequence < 2))
                ctx->is_capture_yuv = 1;
        }

        if (ctx->is_capture_yuv) {
            if (!ctx->_is_yuv_dir_exist) {
                creat_yuv_dir(DEFAULT_CAPTURE_RAW_PATH, ctx);
            }

            if (ctx->_is_yuv_dir_exist) {
                write_yuv_to_file(p, size, sequence, ctx);
            }

            if (ctx->capture_yuv_num-- == 0) {
                ctx->is_capture_yuv = 0;
                ctx->_is_yuv_dir_exist = 0;
            }
        }
    }
}

static int v4l2_read_frame(v4l2_context_t *ctx)
{
    struct v4l2_buffer buf,*v4l2_buf;
    struct v4l2_plane planes[FMT_NUM_PLANES];
    int i, bytesused, v4l2_fd, export_fd;
    int ret = 0;

    v4l2_fd = ctx->fd;
    CLEAR(buf);
    buf.type = ctx->buf_type;
    buf.memory = ctx->memtype?V4L2_MEMORY_DMABUF:V4L2_MEMORY_MMAP;

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
        CLEAR(planes);
        buf.m.planes = planes;
        buf.length = FMT_NUM_PLANES;
    }

    if (-1 == xioctl(v4l2_fd, VIDIOC_DQBUF, &buf)){
        ret = -1;
        errno_exit(ctx, "VIDIOC_DQBUF");
    }

    i  = buf.index;
    export_fd = ctx->buffers[i].export_fd;
    v4l2_buf = &buf;

    if(ctx->log_interval > 0x80000000)
        ctx->log_interval &= ~0x80000000;
    if(buf.sequence == 1 || buf.sequence%ctx->log_interval == 0)
        ctx->log_interval |= 0x80000000;

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type)
        bytesused = buf.m.planes[0].bytesused;
    else
        bytesused = buf.bytesused;

    if (ctx->format == V4L2_PIX_FMT_MJPEG ||
            ctx->format == V4L2_PIX_FMT_H264 ||
            ctx->format == V4L2_PIX_FMT_H265)
        ctx->buffers[i].length = bytesused;

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
        if(ctx->log_interval > 0x80000000)
            DBG("%s: fd-[%d] -> v4l2_plane [bytesused %d, length %d,"
              "mem_offset %d,data_offset %d], frame [%d]\n",
              ctx->dev_name, export_fd, buf.m.planes[0].bytesused, buf.m.planes[0].length,
              buf.m.planes[0].m.mem_offset, buf.m.planes[0].data_offset,buf.sequence);
    }else{
        if(ctx->log_interval > 0x80000000)
            DBG("%s: fd-[%d] -> v4l2_buffer [type: %d, bytesused %d, mem_offset %d, length %d] \n ",
              ctx->dev_name, export_fd, buf.type, buf.bytesused, buf.m.offset, buf.length);
    }

    if(ctx->writeFile || ctx->writeFileSync){
        process_image(ctx->buffers[i].start,  buf.sequence, bytesused, ctx);
        if(!ctx->vop){
            if (-1 == xioctl(v4l2_fd, VIDIOC_QBUF, v4l2_buf)){
                ret = -1;
                errno_exit(ctx, "VIDIOC_QBUF");
            }
        }
    }

    if(ctx->vop == 1){
#ifdef V4L2_BUILD_DAEMO
        int rate_control = 0;
        std::shared_ptr<byteview::VideoFrame> videoFrame = std::make_shared<byteview::VideoFrame>(
            (int)ctx->width, (int)ctx->height, ctx->stride_x, ctx->stride_y, frmtype, "0");
        videoFrame->setFd(export_fd);
        rate_control = ctx->log_interval>0x80000000?1:0;
        videoFrame->setReleaseFunction([v4l2_fd,export_fd,v4l2_buf,rate_control]
            {
                if (rate_control)
                    DBG("release frame fd:%d \n", export_fd);
                if (-1 == xioctl(v4l2_fd, VIDIOC_QBUF, v4l2_buf))
                    ERR("videoFrame QBUF error %d, %s\n", errno, strerror(errno));
            }
        );

        v4l2_videoRenderer->updateFrame(videoFrame);
#else
        ret = i;
#endif
    }

    if(!ctx->vop && !ctx->writeFile && !ctx->writeFileSync){
        if (-1 == xioctl(v4l2_fd, VIDIOC_QBUF, v4l2_buf)){
            ret = -1;
            errno_exit(ctx, "VIDIOC_QBUF");
        }
    }

    return ret;
}

int v4l2_release_frame(v4l2_context *ctx, int index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[FMT_NUM_PLANES];

    CLEAR(buf);
    buf.type = ctx->buf_type;
    buf.memory = ctx->memtype?V4L2_MEMORY_DMABUF:V4L2_MEMORY_MMAP;

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
        CLEAR(planes);
        buf.m.planes = planes;
        buf.length = FMT_NUM_PLANES;
    }

    buf.index = index;

    if (-1 == xioctl(ctx->fd, VIDIOC_QBUF, &buf)){
        errno_exit(ctx, "VIDIOC_QBUF");
        return -1;
    }

    if(ctx->log_interval > 0x80000000)
        DBG("%s: release frame fd:%d \n",ctx->dev_name, ctx->buffers[index].export_fd);

    return 0;
}

static void mainloop(v4l2_context_t *ctx)
{
    int ret = 0;

    while ((ctx->frame_count == -1) || (ctx->frame_count-- > 0))
       v4l2_read_frame(ctx);
}

static int v4l2_check_fmt(v4l2_context_t *ctx)
{
    struct v4l2_format fmt;
    int i = 0, j = 0;
    int w = 0, h = 0;
    int found = -1;

    for(i = 0; i < ctx->nv_infos; i++){
        if(ctx->format == ctx->v_infos[i].pixelformat){
            found = 0;
            if(ctx->v_infos[i].fmt_type == V4L2_FRMSIZE_TYPE_DISCRETE){
                for(j=0; j < ctx->v_infos[i].fmt_cnts; j++) {
                    w = ((struct v4l2_frmsize_discrete *)ctx->v_infos[i].fmt_info)[j].width;
                    h = ((struct v4l2_frmsize_discrete *)ctx->v_infos[i].fmt_info)[j].height;
                    if(w == ctx->width && h == ctx->height){
                        found = 1;
                        break;
                    }
                }
            }else if(ctx->v_infos[i].fmt_type == V4L2_FRMSIZE_TYPE_STEPWISE){
                int ws,hs,w_c,h_c,w_y,h_y;
                w = ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].min_width;
                ws = ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].max_width;
                h = ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].min_height;
                hs = ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].max_height;
                w_y = ctx->width%16;
                h_y = ctx->height%9;
                if(w_y == 0 && h_y == 0 &&
                    ctx->width > w && ctx->width < ws &&
                    ctx->height > h && ctx->height < hs){
                    found = 1;
                    break;
                }
            }
            break;
        }
    }

    fmt.type = ctx->buf_type;
    if(ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){
        fmt.fmt.pix_mp.width  = ctx->width;
        fmt.fmt.pix_mp.height = ctx->height;
        fmt.fmt.pix_mp.pixelformat = ctx->format;
        fmt.fmt.pix_mp.num_planes = 1;
    }else if(ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE){
        fmt.fmt.pix.width  = ctx->width;
        fmt.fmt.pix.height = ctx->height;
        fmt.fmt.pix.pixelformat = ctx->format;
    }
    if (ioctl(ctx->fd, VIDIOC_TRY_FMT, &fmt) == -1){
        errno_exit(ctx,"VIDIOC_TYR_FMT");
        found = -1;
    }

    if(found == -1)
        DBG("[Error]: the pixel format[0x%x] is not supported by camera!\n",ctx->format);
    else if(found == 0)
        DBG("[Warning]: the width&height setting is not standard config for camera!\n");
    else if(found == 1)
        DBG("[%s]: current setting is suitable for camera!\n",__FUNCTION__);

    return found;
}

static int v4l2_enum_fmt(v4l2_context_t *ctx)
{
    struct v4l2_capability cap;
    struct v4l2_fmtdesc fmtdesc;
    struct v4l2_frmsizeenum frmsize;
    struct v4l2_format fmt;
    int ret = 0;
    int i = 0, j = 0;

    CLEAR(fmtdesc);
    CLEAR(fmt);
    if (-1 == xioctl(ctx->fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            ERR("%s: %s is no V4L2 device\n", ctx->dev_name,
                ctx->dev_name);
        } else {
            errno_exit(ctx, "VIDIOC_QUERYCAP");
        }
    }
    DBG("v4l2 capabilities 0x%x\n",cap.capabilities);
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
            !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
        ERR("%s: is not a video capture device, capabilities: %x\n",
            ctx->dev_name, cap.capabilities);
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        ERR("%s: %s does not support streaming i/o\n", ctx->dev_name,
            ctx->dev_name);
    }
    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
        ctx->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        ctx->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    fmtdesc.index = 0;
    fmtdesc.type = ctx->buf_type;
    while (ioctl(ctx->fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        ctx->nv_infos = fmtdesc.index+1;
        ctx->v_infos = (struct video_fmts *)realloc(ctx->v_infos, ctx->nv_infos*sizeof(struct video_fmts));
        ctx->v_infos[fmtdesc.index].pixelformat = fmtdesc.pixelformat;

        CLEAR(frmsize);
        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        while (ioctl(ctx->fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
            ctx->v_infos[fmtdesc.index].fmt_cnts = frmsize.index+1;
            ctx->v_infos[fmtdesc.index].fmt_type = frmsize.type;
            if(frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE){
                ctx->v_infos[fmtdesc.index].fmt_info =
                    (struct v4l2_frmsize_discrete *)realloc(ctx->v_infos[fmtdesc.index].fmt_info,
                        ctx->v_infos[fmtdesc.index].fmt_cnts*sizeof(struct v4l2_frmsize_discrete));
                    ((struct v4l2_frmsize_discrete *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].width  =
                        frmsize.discrete.width;
                    ((struct v4l2_frmsize_discrete *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].height =
                        frmsize.discrete.height;
            }else if(frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE){
                ctx->v_infos[fmtdesc.index].fmt_info =
                    (struct v4l2_frmsize_stepwise *)realloc(ctx->v_infos[fmtdesc.index].fmt_info,
                        ctx->v_infos[fmtdesc.index].fmt_cnts*sizeof(struct v4l2_frmsize_stepwise));
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].min_width  =
                        frmsize.stepwise.min_width;
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].max_width  =
                        frmsize.stepwise.max_width;
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].min_height =
                        frmsize.stepwise.min_height;
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].max_height =
                        frmsize.stepwise.max_height;
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].step_width =
                        frmsize.stepwise.step_width;
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[fmtdesc.index].fmt_info)[frmsize.index].step_height =
                        frmsize.stepwise.step_height;
            }else{
                DBG("frmsize type: V4L2_FRMSIZE_TYPE_CONTINUOUS\n");
            }
            frmsize.index++;
        }
        fmtdesc.index++;
    }

    for(i=0;i<ctx->nv_infos;i++){
        DBG("pixfmt 0x%x , fmt_cnts %d, fmt_type %d \n",
            ctx->v_infos[i].pixelformat, ctx->v_infos[i].fmt_cnts, ctx->v_infos[i].fmt_type);
        for(j = 0; j < ctx->v_infos[i].fmt_cnts; j++){
            if(ctx->v_infos[i].fmt_type == V4L2_FRMSIZE_TYPE_DISCRETE){
                DBG("\t [%d] width %d, height %d \n", j,
                    ((struct v4l2_frmsize_discrete *)ctx->v_infos[i].fmt_info)[j].width,
                    ((struct v4l2_frmsize_discrete *)ctx->v_infos[i].fmt_info)[j].height);
            }else if(ctx->v_infos[i].fmt_type == V4L2_FRMSIZE_TYPE_STEPWISE){
                DBG("\t [%d] width [%d-%d], height [%d-%d], step [%d-%d] \n", j,
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].min_width,
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].max_width,
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].min_height,
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].max_height,
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].step_width,
                    ((struct v4l2_frmsize_stepwise *)ctx->v_infos[i].fmt_info)[j].step_height);
            }
        }
    }

    return ret;
}

static int v4l2_set_ctrls(v4l2_context_t *ctx, enum Cam_Ctrl type, int status)
{
    int ret = 0;
    static int bright = 0, contrast = 0, sharpness = 0, gamma = 0;
    float bright_coef = 0, contrast_coef = 0, sharpness_coef = 0, gamma_coef = 0;
    int cmd = 0, dir = 0;
    struct v4l2_control ctl;
    const char *str = NULL;

    switch (type){
        case CAM_MODE_DEFAULT:
            bright_coef = 2.1;
            contrast_coef = 2.1;
            sharpness_coef = 5.0;
            gamma_coef = 3.1;
            str = "default";
            break;
        case CAM_MODE_SOFT:
            bright_coef = 2.1;
            contrast_coef = 2.1;
            sharpness_coef = 7.5;
            gamma_coef = 1.2;
            str = "soft";
            break;
        case CAM_MODE_STANDARD:
            bright_coef = 2.1;
            contrast_coef = 2.1;
            sharpness_coef =7.5;
            gamma_coef = 3.1;
            str = "standard";
            break;
        case CAM_MODE_CLEAR:
            bright_coef = 2.1;
            contrast_coef = 2.1;
            sharpness_coef = 2.5;
            gamma_coef = 3.1;
            str = "clear";
            break;
        case CAM_MODE_BRIGHT:
            bright_coef = 2.1;
            contrast_coef = 1.6;
            sharpness_coef = 7.5;
            gamma_coef = 3.1;
            str = "bright";
            break;
        case CAM_MODE_COMPUTER:
            bright_coef = 3.0;
            contrast_coef = 1.8;
            sharpness_coef = 7.5;
            gamma_coef = 1.4;
            str = "computer";
            break;
        case CAM_MODE_CLEAR_LED:
            bright_coef = 2.1;
            contrast_coef = 1.8;
            sharpness_coef = 5.0;
            gamma_coef = 1.7;
            str = "clear_led";
            break;
        case CAM_MODE_FACIAL_FEATURES:
            bright_coef = 2.1;
            contrast_coef = 1.8;
            sharpness_coef = 7.5;
            gamma_coef = 1.7;
            str = "facial features";
            break;
        case CAM_CTRL_PAN_LEFT:
            dir = 1;
            cmd = V4L2_CID_PAN_SPEED;
            str = "pan left";
            break;
        case CAM_CTRL_PAN_RIGHT:
            dir = -1;
            cmd = V4L2_CID_PAN_SPEED;
            str = "pan right";
            break;
        case CAM_CTRL_TILT_UP:
            dir = 1;
            cmd = V4L2_CID_TILT_SPEED;
            str = "tilt up";
            break;
        case CAM_CTRL_TILT_DOWN:
            dir = -1;
            cmd = V4L2_CID_TILT_SPEED;
            str = "tilt down";
            break;
        case CAM_CTRL_ZOOM_IN:
            dir = 1;
            cmd = V4L2_CID_ZOOM_CONTINUOUS;
            str = "zoom in";
            break;
        case CAM_CTRL_ZOOM_OUT:
            dir = -1;
            cmd = V4L2_CID_ZOOM_CONTINUOUS;
            str = "zoom out";
            break;
        defaut:
            ERR("video ctrl set -> unknow cmd : %d\n", type);
            goto exit;
    }

    DBG("VIDIOC S_CTRL [%s - %d] \n", str, status);
    if(!cmd){
        int index = 0;
        int b = 0, c = 0, s = 0, g = 0;
        vidctl_node *vnode = NULL;

        index = V4L2_CID_BRIGHTNESS%HASH_TABLE_NUM;
        if((vnode = ctx->vidctl[index]) == NULL)
            ERR("V4L2_CID_BRIGHTNESS ctrl not support!\n");
        while(vnode != NULL){
            if(vnode->key == V4L2_CID_BRIGHTNESS){
                b  = (int) vnode->range/bright_coef;
                break;
            }else{
                vnode = vnode->next;
            }
        }
        index = V4L2_CID_CONTRAST%HASH_TABLE_NUM;
        if((vnode = ctx->vidctl[index]) == NULL)
            ERR("V4L2_CID_CONTRAST ctrl not support!\n");
        while(vnode != NULL){
            if(vnode->key == V4L2_CID_CONTRAST){
                c = (int) vnode->range/contrast_coef;
                break;
            }else{
                vnode = vnode->next;
            }
        }
        index = V4L2_CID_SHARPNESS%HASH_TABLE_NUM;
        if((vnode = ctx->vidctl[index]) == NULL)
            ERR("V4L2_CID_SHARPNESS ctrl not support!\n");
        while(vnode != NULL){
            if(vnode->key == V4L2_CID_SHARPNESS){
                s = (int) vnode->range/sharpness_coef;
                break;
            }else{
                vnode = vnode->next;
            }
        }
        index = V4L2_CID_GAMMA%HASH_TABLE_NUM;
        if((vnode = ctx->vidctl[index]) == NULL)
            ERR("V4L2_CID_GAMMA ctrl not support!\n");
        while(vnode != NULL){
            if(vnode->key == V4L2_CID_GAMMA){
                g = (int) vnode->range/gamma_coef;
                g += vnode->min;
                break;
            }else{
                vnode = vnode->next;
            }
        }

        DBG("camera view mode config [%d,%d,%d,%d]!\n",b,c,s,g);
        if(bright != b){
            bright = b;
            ctl.id = V4L2_CID_BRIGHTNESS;
            ctl.value = bright;
            if((ret = xioctl(ctx->fd, VIDIOC_S_CTRL, &ctl)) == -1)
                errno_exit(ctx,"VIDIOC_S_CTRL for brightness");
        }
        if(contrast != c){
            contrast = c;
            ctl.id = V4L2_CID_CONTRAST;
            ctl.value = contrast;
            if((ret = xioctl(ctx->fd, VIDIOC_S_CTRL, &ctl)) == -1)
                errno_exit(ctx,"VIDIOC_S_CTRL for contrast");
        }
        if(sharpness != s){
            sharpness = s;
            ctl.id = V4L2_CID_SHARPNESS;
            ctl.value = sharpness;
            if((ret = xioctl(ctx->fd, VIDIOC_S_CTRL, &ctl)) == -1)
                errno_exit(ctx,"VIDIOC_S_CTRL for sharpness");
        }
        if(gamma != g){
            gamma = g;
            ctl.id = V4L2_CID_GAMMA;
            ctl.value = gamma;
            if((ret = xioctl(ctx->fd, VIDIOC_S_CTRL, &ctl)) == -1)
                errno_exit(ctx,"VIDIOC_S_CTRL for gamma");
        }
    }else{
        int index = 0;
        int speed = 0;
        vidctl_node *vnode = NULL;

        index = cmd%HASH_TABLE_NUM;
        if((vnode = ctx->vidctl[index]) == NULL){
            ERR("camera ptz-ctrl[0x%x] not support!\n",cmd);
            speed = 0xffffffff;
        }
        while(vnode != NULL){
            if(vnode->key == cmd){
                if(dir > 0)
                    speed = vnode->max;
                else
                    speed = vnode->min;
                break;
            }else{
                vnode = vnode->next;
            }
        }
        if(speed != 0xffffffff){
            if(status == 0)
                ctl.value = 0;
            else
                ctl.value = speed*2/3;
            DBG("camera ptz adjust [0x%x][%s][%d-%d] !\n",cmd ,status>0?"start":"stop", speed, ctl.value);
            ctl.id = cmd;
            if(xioctl(ctx->fd, VIDIOC_S_CTRL, &ctl) == -1)
                errno_exit(ctx,"VIDIOC_S_CTRL for camera ptz");
        }
    }

exit:
    return ret;
}

static int v4l2_enum_ctrls(v4l2_context_t *ctx)
{
    struct v4l2_queryctrl queryctrl;
    struct v4l2_query_ext_ctrl extqueryctrl;
    int i = 0;

    queryctrl.id = V4L2_CID_BASE;
    while (xioctl(ctx->fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
        int index = 0;

        DBG("Name: %s ID: 0x%x Type: %d Minimum: %d Maximum: %d Default: %d Flags: %u Step: %d \n",
            queryctrl.name, queryctrl.id, queryctrl.type, queryctrl.minimum,
            queryctrl.maximum, queryctrl.default_value, queryctrl.flags, queryctrl.step);
        index = queryctrl.id % HASH_TABLE_NUM;
        vidctl_node *vnode = (vidctl_node *)malloc(sizeof(vidctl_node));
        vnode->key = queryctrl.id;
        vnode->min = queryctrl.minimum;
        vnode->max = queryctrl.maximum;
        vnode->range = queryctrl.maximum-queryctrl.minimum+1;
        if (ctx->vidctl[index] == NULL){
            ctx->vidctl[index] = vnode;
        }else{
            vidctl_node *cur = ctx->vidctl[index];
            while(cur->next != NULL){
                cur = cur->next;
            }
            cur->next = vnode;
        }
        queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }

    for(int i=0; i < HASH_TABLE_NUM; i++) {
        vidctl_node *vnode = ctx->vidctl[i];
        while(vnode !=NULL) {
            DBG("id 0x%x, min %d, max %d, range %d \n",
                vnode->key, vnode->min,
                vnode->max, vnode->range);
            vnode = vnode->next;
        }
    }

#if 0
    extqueryctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
    while (xioctl(ctx->fd, VIDIOC_QUERY_EXT_CTRL, &extqueryctrl) == 0) {
        DBG("Name: %s\n", extqueryctrl.name);
        DBG("ID: 0x%x\n", extqueryctrl.id);
        DBG("Type: %d\n", extqueryctrl.type);
        DBG("Minimum: %ld\n", extqueryctrl.minimum);
        DBG("Maximum: %ld\n", extqueryctrl.maximum);
        DBG("Default: %ld\n", extqueryctrl.default_value);

        DBG("ELEMS: %d\n", extqueryctrl.elems);
        DBG("ELEM_SIZE: %d\n", extqueryctrl.elem_size);
        DBG("ELEM_NUM: %d\n", extqueryctrl.nr_of_dims);
        DBG("DIMS: %d %d %d %d \n",
                extqueryctrl.dims[0],
                extqueryctrl.dims[1],
                extqueryctrl.dims[2],
                extqueryctrl.dims[3]);

        extqueryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
#endif

  return 0;
}

static void stop_capturing(v4l2_context_t *ctx)
{
    enum v4l2_buf_type type;

    type = ctx->buf_type;
    if (-1 == xioctl(ctx->fd, VIDIOC_STREAMOFF, &type))
        errno_exit(ctx, "VIDIOC_STREAMOFF");
}

static void start_capturing(v4l2_context_t *ctx)
{
    struct v4l2_plane planes[FMT_NUM_PLANES];
    struct v4l2_buffer buf;
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < ctx->n_buffers; ++i) {
        CLEAR(buf);
        buf.type = ctx->buf_type;
        buf.memory = ctx->memtype? V4L2_MEMORY_DMABUF:V4L2_MEMORY_MMAP;
        buf.index = i;

        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
            CLEAR(planes);
            buf.m.planes = planes;
            buf.length = FMT_NUM_PLANES;

            if(ctx->memtype)
                buf.m.planes[0].m.fd = ctx->buffers[i].export_fd;
        }else if(V4L2_BUF_TYPE_VIDEO_CAPTURE == ctx->buf_type){
            buf.m.fd = ctx->buffers[i].export_fd;
        }

        DBG("[ctx->buffers] start 0x%lx, length %ld, handle %d, export_fd %d\n",
            (long)ctx->buffers[i].start, ctx->buffers[i].length,
            ctx->buffers[i].handle, ctx->buffers[i].export_fd);

        if (-1 == xioctl(ctx->fd, VIDIOC_QBUF, &buf))
            errno_exit(ctx, "VIDIOC_QBUF");
    }
    type = ctx->buf_type;
    DBG("%s:-------- stream on output -------------\n", ctx->dev_name);

    if (-1 == xioctl(ctx->fd, VIDIOC_STREAMON, &type))
        errno_exit(ctx, "VIDIOC_STREAMON");
}

static void init_mmap(v4l2_context_t *ctx)
{
    struct v4l2_requestbuffers req;
    struct buffer *tmp_buffers = NULL;

    CLEAR(req);
    req.count = BUFFER_COUNT;
    req.type = ctx->buf_type;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(ctx->fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            ERR("%s does not support "
                "memory mapping\n", ctx->dev_name);
        } else {
            errno_exit(ctx, "VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        ERR("%s: Insufficient buffer memory on %s\n", ctx->dev_name,
            ctx->dev_name);
    }

    tmp_buffers = (struct buffer*)calloc(req.count, sizeof(struct buffer));

    if (!tmp_buffers) {
        ERR("%s: Out of memory\n", ctx->dev_name);
    }

    ctx->buffers = tmp_buffers;

    for (ctx->n_buffers = 0; ctx->n_buffers < req.count; ++ctx->n_buffers) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[FMT_NUM_PLANES];
        CLEAR(buf);
        CLEAR(planes);

        buf.type = ctx->buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = ctx->n_buffers;

        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
            buf.m.planes = planes;
            buf.length = FMT_NUM_PLANES;
        }

        if (-1 == xioctl(ctx->fd, VIDIOC_QUERYBUF, &buf))
            errno_exit(ctx, "VIDIOC_QUERYBUF");

        if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
            DBG("v4l2_buffer [type: %d, bytesused %d], "
                "v4l2_plane [bytesused %d, length %d, mem_offset %d ,data_offset %d]\n",
                buf.type, buf.bytesused,buf.m.planes[0].bytesused, buf.m.planes[0].length,
                buf.m.planes[0].m.mem_offset, buf.m.planes[0].data_offset);
            tmp_buffers[ctx->n_buffers].length = buf.m.planes[0].length;
            tmp_buffers[ctx->n_buffers].start =
                mmap(NULL /* start anywhere */,
                     buf.m.planes[0].length,
                     PROT_READ | PROT_WRITE /* required */,
                     MAP_SHARED /* recommended */,
                     ctx->fd, buf.m.planes[0].m.mem_offset);
        } else {
            DBG("v4l2_buffer [type: %d, bytesused %d, mem_offset %d, length %d] \n ",
                buf.type, buf.bytesused,buf.m.offset, buf.length);
            tmp_buffers[ctx->n_buffers].length = buf.length;
            tmp_buffers[ctx->n_buffers].start =
                mmap(NULL /* start anywhere */,
                     buf.length,
                     PROT_READ | PROT_WRITE /* required */,
                     MAP_SHARED /* recommended */,
                     ctx->fd, buf.m.offset);
        }

        if (MAP_FAILED == tmp_buffers[ctx->n_buffers].start)
            errno_exit(ctx, "mmap");

        // export buf dma fd
        struct v4l2_exportbuffer expbuf;
        memset(&expbuf,0,sizeof(expbuf));
        expbuf.type = ctx->buf_type;
        expbuf.index = ctx->n_buffers;
        expbuf.flags = O_CLOEXEC;
        if (xioctl(ctx->fd, VIDIOC_EXPBUF, &expbuf) < 0) {
            errno_exit(ctx, "get dma buf failed\n");
        } else {
            DBG("%s: get dma buf(%d)-fd: %d\n", ctx->dev_name, ctx->n_buffers, expbuf.fd);
        }
        tmp_buffers[ctx->n_buffers].export_fd = expbuf.fd;
    }
}

static void deinit_device(v4l2_context_t *ctx)
{
    unsigned int i;

    if (ctx->n_buffers == 0)
        return;

    for (i = 0; i < ctx->n_buffers; ++i) {
        if (-1 == munmap(ctx->buffers[i].start, ctx->buffers[i].length))
            errno_exit(ctx, "munmap");
        close(ctx->buffers[i].export_fd);
        if(ctx->memtype)
            drm_free(ctx->drm_fd, &ctx->buffers[i]);
    }

    free(ctx->buffers);
    ctx->n_buffers = 0;
}

static void init_device(v4l2_context_t *ctx)
{
    struct v4l2_streamparm param;
    struct v4l2_format fmt;

    CLEAR(param);
    CLEAR(fmt);

    if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE){
        fmt.fmt.pix.width = ctx->width;
        fmt.fmt.pix.height = ctx->height;
        fmt.fmt.pix.pixelformat = ctx->format;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
    } else if (ctx->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE){
        fmt.type = ctx->buf_type;
        fmt.fmt.pix_mp.width = ctx->width;
        fmt.fmt.pix_mp.height = ctx->height;
        fmt.fmt.pix_mp.pixelformat = ctx->format;
        fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    }
    fmt.type = ctx->buf_type;
    if (ctx->limit_range)
        fmt.fmt.pix.quantization = V4L2_QUANTIZATION_LIM_RANGE;
    else
        fmt.fmt.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;

    if (-1 == xioctl(ctx->fd, VIDIOC_S_FMT, &fmt))
        errno_exit(ctx, "VIDIOC_S_FMT");

    DBG("Pixel[format 0x%x, size %d], Mode[%s] \n",ctx->format,
        fmt.fmt.pix.sizeimage,ctx->buf_type==V4L2_BUF_TYPE_VIDEO_CAPTURE?
        "V4L2_CAP_VIDEO_CAPTURE":"V4L2_CAP_VIDEO_CAPTURE_MPLANE");

    if(ctx->srcfps != 0){
        param.type = ctx->buf_type;
        param.parm.capture.timeperframe.numerator = 1;
        param.parm.capture.timeperframe.denominator = ctx->srcfps;
        param.parm.capture.capturemode = ctx->buf_type;

        if (-1 == xioctl(ctx->fd, VIDIOC_S_PARM, &param))
            errno_exit(ctx, "VIDIOC_S_PARM");
    }

    if(ctx->memtype)
        drm_init_mmap(ctx);
    else
        init_mmap(ctx);
}

static void close_device(v4l2_context_t *ctx)
{
    int i = 0;

    for(int i=0; i < HASH_TABLE_NUM; i++) {
        vidctl_node *vnode = ctx->vidctl[i];
        while(vnode != NULL) {
            vidctl_node *tmp = vnode;
            vnode = vnode->next;
            free(tmp);
        }
    }

    if(ctx->nv_infos) {
        for(i=0; i < ctx->nv_infos; i++)
            free(ctx->v_infos[i].fmt_info);
        free(ctx->v_infos);
    }

    if(ctx->pipe) {
        ctx->pipe = 0;
        unlink(p_fifo);
        pthread_join(camera_control, NULL);
    }

    if(ctx->vop == 1) {
#ifdef V4L2_BUILD_DAEMO
        deinit_render(ctx);
#endif
    }

    if (ctx->writeFile) {
        fclose(ctx->fp);
        ctx->fp = NULL;
    }

    if(ctx->memtype) {
        drm_close(ctx->drm_fd);
        ctx->drm_fd = -1;
    }

    if (-1 == close(ctx->fd))
        errno_exit(ctx, "close");

    ctx->fd = -1;
}

static void open_device(v4l2_context_t *ctx)
{
    DBG("-------- open output dev_name:%s -------------\n", ctx->dev_name);
    ctx->fd = open(ctx->dev_name, O_RDWR | O_CLOEXEC /* required */ /*| O_NONBLOCK*/, 0);
    if (-1 == ctx->fd)
        errno_exit(ctx, "erro open v4l2dev\n");

    if (ctx->memtype){
        ctx->drm_fd = drm_open();
        if (ctx->drm_fd == -1)
            errno_exit(ctx, "drm open failed");
    }

    if (ctx->writeFile) {
        ctx->fp = fopen(ctx->out_file, "w+");
        if (ctx->fp == NULL) {
            ERR("%s: fopen output file %s failed!\n", ctx->dev_name, ctx->out_file);
        }
    }

    if(ctx->vop == 1){
#ifdef V4L2_BUILD_DAEMO
        init_render(ctx);
#endif
    }

    if(ctx->pipe){
        mkfifo(p_fifo, 0666);
        pthread_create(&camera_control, NULL, &camera_thread, ctx);
    }

    v4l2_enum_fmt(ctx);
    v4l2_enum_ctrls(ctx);

    ctx->stride_y = (ctx->height+15)&~15;
    switch(ctx->format) {
    default:
        ERR("fix me!! using nv12 stride by default \n");
    case V4L2_PIX_FMT_NV12:
    case V4L2_PIX_FMT_NV21:
    case V4L2_PIX_FMT_NV16:
    case V4L2_PIX_FMT_NV61:
        ctx->stride_x = ctx->width;
        break;
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YVYU:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_VYUY:
        // fix me!! actually 2*width
        ctx->stride_x = ctx->width;
        break;
    }
    ctx->log_interval = ctx->log_interval==0?1:30*ctx->log_interval;
    DBG("frame log_interval every %d frames \n",ctx->log_interval);
}

void v4l2_deinit(v4l2_context_t *ctx)
{
    stop_capturing(ctx);
    deinit_device(ctx);
}

void v4l2_routine(v4l2_context_t *ctx)
{
    init_device(ctx);
    start_capturing(ctx);
}

void *camera_thread(void *arg)
{
    v4l2_context_t *ctx = (v4l2_context_t *)arg;
    char buf[64] = {0};
    int fd = -1;
    int len = 0;

    fd = open(p_fifo, O_RDONLY|O_NONBLOCK|O_CLOEXEC);
    if(fd < 0){
        errno_exit(ctx,"open fifo error");
        goto rtn;
    }
    while(ctx->pipe){
        memset(buf, 0, sizeof(buf));
        len = read(fd, buf, sizeof(buf));
        if(len > 0){
            int type = -1,cmd = -1;
            char *token;
            char *str = buf;

            DBG("len-> %zu , cmd-> %s",strlen(buf),buf);
            if(!memcmp(buf,"mode",4)){
                type = 0;
            }else if(!memcmp(buf,"ctrl",4)){
                type = 1;
            }else{
                ERR("unknow cmd!");
                continue;
            }
            token = strsep(&str,":");
            token = strsep(&str,":");
            enum Cam_Ctrl mode = (enum Cam_Ctrl)atoi(token);
            if(type){
                token = strsep(&str,":");
                cmd = (enum Cam_Ctrl)atoi(token);
            }

            if(type == 0) {
                DBG("view mode config !\n");
                if(mode > CAM_MODE_FACIAL_FEATURES){
                    ERR("camera view mode unknow [%d] !\n",mode);
                    continue;
                }
                v4l2_set_ctrls(ctx, mode, 1);
            }else if (type == 1) {
                DBG("ptz ctrl config !\n");
                if(mode > CAM_CTRL_ZOOM_OUT){
                    ERR("camera ctrl type unknow [%c] !\n",mode);
                    continue;
                } else if (cmd != 0 && cmd != 1) {
                    ERR("camera ctrl cmd unknow [%c] !\n",cmd);
                    continue;
                }
                v4l2_set_ctrls(ctx, mode, cmd);
            }else{
                ERR("cmd bad format !\n");
            }
        }else{
            if(len == -1 && errno != EAGAIN && errno != EINTR)
                ERR("error %d, %s\n", errno, strerror(errno));
            usleep(60*1000);
        }
    }

rtn:
    close(fd);
    pthread_exit(NULL);
}

#ifdef V4L2_BUILD_DAEMO
static void init_render(v4l2_context_t *ctx)
{
    byteview::VideoRenderConfig c = byteview::VideoRenderConfig(byteview::VideoRenderType::tEGL);
    c.setMediaDemo(true);
    c.setZindex(1);

    std::shared_ptr<byteview::VideoRenderer> videoRender = byteview::VideoRenderer::create(c);
    v4l2_videoRenderer = videoRender;

    if (ctx->format == V4L2_PIX_FMT_NV12)
        frmtype = byteview::FrameType::kNv12_OESFD;
    else if (ctx->format == V4L2_PIX_FMT_NV21)
        frmtype = byteview::FrameType::kNv21_OESFD;
    else if (ctx->format == V4L2_PIX_FMT_NV16)
        frmtype = byteview::FrameType::kNv16_OESFD;
    else if (ctx->format == V4L2_PIX_FMT_YUYV)
        frmtype = byteview::FrameType::kYUYV_OESFD;
    else
        DBG("unsupported format,render default using nv12 \n");

}

static void deinit_render(v4l2_context_t *ctx)
{
    v4l2_videoRenderer->quit();
    v4l2_videoRenderer = NULL;
}

static void parse_args(int argc, char **argv, v4l2_context_t *ctx)
{
    int c;
    int digit_optind = 0;
    optind = 0;
    while (1) {
        int this_option_optind = optind ? optind : 1;
        int option_index = 0;
        static struct option long_options[] = {
            {"width",    required_argument, 0, 'w' },
            {"height",   required_argument, 0, 'h' },
            {"format",   required_argument, 0, 'f' },
            {"srcfps",   required_argument, 0, 'i' },
            {"dstfps",   required_argument, 0, 't' },
            {"device",   required_argument, 0, 'd' },
            {"memtype",  required_argument, 0, 'm' },
            {"stream-to",     required_argument, 0, 'o' },
            {"stream-count",  required_argument, 0, 'n' },
            {"stream-skip",   required_argument, 0, 'k' },
            {"hdr",           required_argument, 0, 'a' },
            {"count",         required_argument, 0, 'c' },
            {"log-interval",  required_argument, 0, 'g' },
            {"sync-to-raw",   no_argument,  0, 'e' },
            {"vop",      no_argument      , 0, 'v' },
            {"silent",   no_argument,       0, 's' },
            {"limit",    no_argument,       0, 'l' },
            {"pipe",     no_argument,       0, 'y' },
            {"help",     no_argument,       0, 'p' },
            {0,          0,                 0,  0  }
        };

        //c = getopt_long(argc, argv, "w:h:f:i:d:o:c:ps",
        c = getopt_long(argc, argv, "w:h:f:i:t:m:vd:o:n:k:c:a:psely",
                        long_options, &option_index);
        if (c == -1)
            break;
        switch (c) {
        case 'c':
            ctx->frame_count = atoi(optarg);
            break;
        case 'w':
            ctx->width = atoi(optarg);
            break;
        case 'h':
            ctx->height = atoi(optarg);
            break;
        case 'i':
            ctx->srcfps = atoi(optarg);
            break;
        case 't':
            ctx->dstfps = atoi(optarg);
            break;
        case 'f':
            ctx->format = v4l2_fourcc(optarg[0], optarg[1], optarg[2], optarg[3]);
            break;
        case 'd':
            strcpy(ctx->dev_name, optarg);
            break;
        case 'm':
            ctx->memtype = atoi(optarg);
            break;
        case 'v':
            ctx->vop = 1;
            break;
        case 'y':
            ctx->pipe = 1;
            break;
        case 'o':
            strcpy(ctx->out_file, optarg);
            ctx->writeFile = 1;
            break;
        case 'n':
            ctx->outputCnt = atoi(optarg);
            break;
        case 'k':
            ctx->skipCnt = atoi(optarg);
            break;
        case 's':
            silent = 1;
            break;
        case 'a':
            ctx->hdrmode = atoi(optarg);
            break;
        case 'e':
            ctx->writeFileSync = 1;
            break;
        case 'l':
            ctx->limit_range = 1;
            break;
        case 'g':
            ctx->log_interval = 30*atoi(optarg);
            break;
        default:
            ERR("?? getopt returned character code 0%o ??\n", c);
        case '?':
        case 'p':
            goto usage;
        }
    }

    if (strlen(ctx->dev_name) == 0) {
        ERR("[error]: --device must be configured \n");
        goto usage;
    }

    return ;
usage:
    ERR("Usage: %s to capture frames\n"
        "         --width,  default 640,             optional, width of image\n"
        "         --height, default 360,             optional, height of image\n"
        "         --srcfps, default 30,              optional, source fps of camera\n"
        "         --dstfps, default 30,              optional, output fps of camera\n"
        "         --format, default NV12,            optional, fourcc of format\n"
        "         --count,  default -1,              optional, how many frames to capture, default unlimited\n"
        "         --device,                          required, path of video device\n"
        "         --memtype,                         optional, 0:v4l2 fd, 1:drm fd, default 0\n"
        "         --vop,                             optional, frame render out\n"
        "         --pipe,                            optional, create fifo pipe thread for rpc\n"
        "         --stream-to,                       optional, output file path, if <file> is '-', then the data is written to stdout\n"
        "         --stream-count, default 60         optional, how many frames to write files\n"
        "         --stream-skip, default 30          optional, how many frames to skip befor writing file\n"
        "         --silent,                          optional, subpress debug log\n"
        "         --hdr <val>,                       optional, hdr mode, val 2 means hdrx2, 3 means hdrx3 \n"
        "         --sync-to-raw,                     optional, write yuv files in sync with raw\n"
        "         --limit,                           optional, yuv limit range\n"
        "         --log-interval,                    optional, frame log print intervals (second)\n"
        "example:\n"
        "         byted_lark_v4l2 -w 1920 -h 1080 -f NV12 -c 120 -d /dev/video11 -m 0 -v \n"
        ,argv[0]);
    exit(-1);
}

static void signal_handle(int signo)
{
    DBG("force exit signo %d !!!\n", signo);

    if (g_main_ctx) {
        g_main_ctx->frame_count = 0;
        v4l2_deinit(g_main_ctx);
        close_device(g_main_ctx);
        g_main_ctx = NULL;
    }

    exit(0);
}

int main(int argc, char **argv)
{
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    sigaddset(&mask, SIGQUIT);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);
    int i = 0;
    char buf[16] = {0};

    struct sigaction new_action, old_action;
    new_action.sa_handler = signal_handle;
    sigemptyset (&new_action.sa_mask);
    new_action.sa_flags = 0;
    sigaction (SIGINT, NULL, &old_action);
    if (old_action.sa_handler != SIG_IGN)
        sigaction (SIGINT, &new_action, NULL);
    sigaction (SIGQUIT, NULL, &old_action);
    if (old_action.sa_handler != SIG_IGN)
        sigaction (SIGQUIT, &new_action, NULL);
    sigaction (SIGTERM, NULL, &old_action);
    if (old_action.sa_handler != SIG_IGN)
        sigaction (SIGTERM, &new_action, NULL);

    v4l2_context_t main_ctx = {
        .out_file = {'\0'},
        .dev_name = {'\0'},
        .width = 640,
        .height = 480,
        .srcfps = 0,
        .dstfps = 0,
        .format = V4L2_PIX_FMT_NV12,
        .memtype = 0,
        .vop = 0,
        .fd = -1,
        .buf_type = V4L2_BUF_TYPE_PRIVATE,
        .buffers = NULL,
        .n_buffers = 0,
        .frame_count = -1,
        .fp = NULL,
        .writeFile = 0,
        .writeFileSync = 0,
        .hdrmode = 0,
        .limit_range = 0,
        .outputCnt = 60,
        .skipCnt = 30,
        .nv_infos = 0,
        .pipe = 0,
        .yuv_dir_path = {'\0'},
        ._is_yuv_dir_exist = 0,
        .capture_yuv_num = 0,
        .is_capture_yuv = 0,
        .log_interval = 0,
    };
    parse_args(argc, argv, &main_ctx);

    /* start up rkaiq service */
    __system_property_set("bytedlark.rkaiq.on", "1");
    while(i++ < 10) {
        __system_property_get("bytedlark.rkaiq.running",buf);
        if(buf[0] == '0') {
            DBG("%s, waiting for rkisp service ready\n",__FUNCTION__);
            usleep(50*1000);
        }else{
            DBG("%s, %s",__FUNCTION__,
                buf[0] == '1'?"rkisp status -> [running]!\n":
                buf[0] == '2'?"rkisp status -> [disabled]!\n":
                "rkisp status -> [unkown]!\n");
            break;
        }
    }
    if(i == 10)
        ERR("%s->[error] waiting for rkisp service timeout!"
          "video capture maybe unnormal!",__FUNCTION__);
    __system_property_set("bytedlark.rkaiq.running","0");

    open_device(&main_ctx);
    if(v4l2_check_fmt(&main_ctx) < 0)
        goto rtn;
    v4l2_routine(&main_ctx);
    g_main_ctx = &main_ctx;
    pthread_sigmask(SIG_UNBLOCK, &mask, NULL);
    mainloop(&main_ctx);
    v4l2_deinit(&main_ctx);
rtn:
    close_device(&main_ctx);

    return 0;
}
#else
#include "video_capture.cpp"
#include "capture_node.cpp"
#endif
