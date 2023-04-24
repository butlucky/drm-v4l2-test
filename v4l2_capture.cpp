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
#include "v4l2/videodev2.h"
#ifdef V4L2_BUILD_DAEMO
#include "../video_render/render/VideoRenderer.h"
#include "../common/video_frame/VideoFrame.h"
static std::shared_ptr<byteview::VideoRenderer> v4l2_videoRenderer;
#endif

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define FMT_NUM_PLANES 1

#define BUFFER_COUNT 6
#define CAPTURE_RAW_PATH "/data"
#define DEFAULT_CAPTURE_RAW_PATH "/data/capture_image"
#define CAPTURE_CNT_FILENAME ".capture_cnt"
#define DBG(...) do { if(!silent) { printf("[%s-%d]--> ",__func__,__LINE__); printf(__VA_ARGS__);} } while(0)
#define ERR(...) do { printf("[%s-%d]--> ",__func__,__LINE__); printf(__VA_ARGS__); } while (0)

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

    char yuv_dir_path[64];
    int _is_yuv_dir_exist;
    int capture_yuv_num;
    int is_capture_yuv;
} v4l2_context_t;

struct buffer {
    void *start;
    long length;
    int export_fd;
    int handle;
    int sequence;
};
static int silent,rate_print;
static v4l2_context_t *g_main_ctx = NULL;
static const char *dev_drm = "/dev/dri/card0";
#ifdef V4L2_BUILD_DAEMO
static void init_render(v4l2_context_t *ctx);
#endif

static void errno_exit(v4l2_context_t *ctx, const char *s)
{
    printf("%s: %s error %d, %s\n", ctx->dev_name, s, errno, strerror(errno));
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
    default:
        ERR("unsupporrt type,using default nv12 format!\n");
        dmcb.bpp    = 8;
        dmcb.height = height*3/2;
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

    fp = open(path, O_RDONLY | O_SYNC);
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
        if(rate_print)
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

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type)
        bytesused = buf.m.planes[0].bytesused;
    else
        bytesused = buf.bytesused;

#ifdef V4L2_BUILD_DAEMO
    rate_print = 1;
#else
    rate_print = 0;
    if(buf.sequence%180 == 0)
        rate_print = 1;
#endif

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->buf_type) {
        if (rate_print)
            DBG("%s: fd-[%d] -> v4l2_plane [bytesused %d, length %d,"
              "mem_offset %d,data_offset %d], frame [%d]\n",
              ctx->dev_name, export_fd, buf.m.planes[0].bytesused, buf.m.planes[0].length,
              buf.m.planes[0].m.mem_offset, buf.m.planes[0].data_offset,buf.sequence);
    }else{
        if(rate_print)
            DBG("%s: fd-[%d] -> v4l2_buffer [type: %d, bytesused %d, mem_offset %d, length %d] \n ",
              ctx->dev_name, export_fd, buf.type, buf.bytesused, buf.m.offset, buf.length);
    }

    if(ctx->vop == 1 && !ctx->writeFileSync){
#ifdef V4L2_BUILD_DAEMO
        std::shared_ptr<byteview::VideoFrame> videoFrame = std::make_shared<byteview::VideoFrame>(
            (int)ctx->width, (int)ctx->height, ctx->stride_x, ctx->stride_y,
            byteview::FrameType::kNv12_OESFD, "0");

        videoFrame->setFd(export_fd);
        v4l2_videoRenderer->updateFrame(videoFrame);

        videoFrame->setReleaseFunction([v4l2_fd,export_fd,v4l2_buf]
            {
                DBG("release frame fd:%d \n", export_fd);
                if (-1 == xioctl(v4l2_fd, VIDIOC_QBUF, v4l2_buf))
                    ERR("videoFrame QBUF error %d, %s\n", errno, strerror(errno));
            }
        );
#else
        ret = i;
#endif
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

    if(rate_print)
        DBG("%s: release frame fd:%d \n",ctx->dev_name, ctx->buffers[index].export_fd);

    return 0;
}

static void mainloop(v4l2_context_t *ctx)
{
    int ret = 0;

    while ((ctx->frame_count == -1) || (ctx->frame_count-- > 0))
       v4l2_read_frame(ctx);
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

static void uninit_device(v4l2_context_t *ctx)
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

    if(ctx->memtype)
        drm_close(ctx->drm_fd);
    free(ctx->buffers);
    ctx->n_buffers = 0;
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

static void init_device(v4l2_context_t *ctx)
{
    struct v4l2_capability cap;
    struct v4l2_streamparm param;
    struct v4l2_format fmt;

    CLEAR(param);
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

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        DBG("v4l2 mode V4L2_CAP_VIDEO_CAPTURE, pix format 0x%x \n",ctx->format);
        ctx->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.type = ctx->buf_type;
        fmt.fmt.pix.width = ctx->width;
        fmt.fmt.pix.height = ctx->height;
        fmt.fmt.pix.pixelformat = ctx->format;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (ctx->limit_range)
            fmt.fmt.pix.quantization = V4L2_QUANTIZATION_LIM_RANGE;
        else
            fmt.fmt.pix.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    } else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        DBG("v4l2 mode V4L2_CAP_VIDEO_CAPTURE_MPLANE, pix format 0x%x \n",ctx->format);
        ctx->buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.type = ctx->buf_type;
        fmt.fmt.pix_mp.width = ctx->width;
        fmt.fmt.pix_mp.height = ctx->height;
        fmt.fmt.pix_mp.pixelformat = ctx->format;
        fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
        if (ctx->limit_range)
            fmt.fmt.pix_mp.quantization = V4L2_QUANTIZATION_LIM_RANGE;
        else
            fmt.fmt.pix_mp.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    }

    if (-1 == xioctl(ctx->fd, VIDIOC_S_FMT, &fmt))
        errno_exit(ctx, "VIDIOC_S_FMT");

    if(ctx->srcfps != 0){
        param.type = ctx->buf_type;
        param.parm.capture.timeperframe.numerator = 1;
        param.parm.capture.timeperframe.denominator = ctx->srcfps;
        param.parm.capture.capturemode = ctx->buf_type;

        if (-1 == xioctl(ctx->fd, VIDIOC_S_PARM, &param))
            errno_exit(ctx, "VIDIOC_S_FMT");
    }

    if(ctx->memtype)
        drm_init_mmap(ctx);
    else
        init_mmap(ctx);

}

static void close_device(v4l2_context_t *ctx)
{
    if (-1 == close(ctx->fd))
        errno_exit(ctx, "close");

    ctx->fd = -1;
}

static void open_device(v4l2_context_t *ctx)
{
    DBG("-------- open output dev_name:%s -------------\n", ctx->dev_name);
    ctx->fd = open(ctx->dev_name, O_RDWR /* required */ /*| O_NONBLOCK*/, 0);

    if (-1 == ctx->fd)
        errno_exit(ctx, "erro open v4l2dev\n");

    if (ctx->memtype == 1){
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

    if(ctx->stride_x == 0)
        ctx->stride_x = ctx->width;
    if(ctx->stride_y == 0)
        ctx->stride_y = ctx->height;
}

void v4l2_deinit(v4l2_context_t *ctx)
{
    stop_capturing(ctx);
    uninit_device(ctx);
    close_device(ctx);

    if (ctx->fp)
        fclose(ctx->fp);
}

static void signal_handle(int signo)
{
    DBG("force exit signo %d !!!\n", signo);

    if (g_main_ctx) {
        g_main_ctx->frame_count = 0;
        v4l2_deinit(g_main_ctx);
        g_main_ctx = NULL;
    }

    exit(0);
}

void v4l2_routine(v4l2_context_t *ctx)
{
    open_device(ctx);
    init_device(ctx);
    start_capturing(ctx);
}

#ifdef V4L2_BUILD_DAEMO
static void init_render(v4l2_context_t *ctx)
{
    byteview::VideoRenderConfig c = byteview::VideoRenderConfig(byteview::VideoRenderType::tEGL);
    c.setMediaDemo(true);
    c.setZindex(1);

    std::shared_ptr<byteview::VideoRenderer> videoRender = byteview::VideoRenderer::create(c);
    v4l2_videoRenderer = videoRender;
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
            {"vop",      no_argument      , 0, 'v' },
            {"stream-to",     required_argument, 0, 'o' },
            {"stream-count",  required_argument, 0, 'n' },
            {"stream-skip",   required_argument, 0, 'k' },
            {"hdr",           required_argument, 0, 'a' },
            {"sync-to-raw",   no_argument,  0, 'e' },
            {"count",    required_argument, 0, 'c' },
            {"help",     no_argument,       0, 'p' },
            {"silent",   no_argument,       0, 's' },
            {"limit",    no_argument,       0, 'l' },
            {0,          0,                 0,  0  }
        };

        //c = getopt_long(argc, argv, "w:h:f:i:d:o:c:ps",
        c = getopt_long(argc, argv, "w:h:f:i:t:m:vd:o:n:k:c:a:psel",
                        long_options, &option_index);
        if (c == -1)
            break;
        switch (c) {
        case 'c':
            ctx->frame_count = atoi(optarg);
            break;
        case 'w':
            ctx->width = atoi(optarg);
            ctx->stride_x = atoi(optarg);
            break;
        case 'h':
            ctx->height = atoi(optarg);
            ctx->stride_y = atoi(optarg);
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
        "         --height, default 480,             optional, height of image\n"
        "         --srcfps, default 30,              optional, source fps of camera\n"
        "         --dstfps, default 30,              optional, output fps of camera\n"
        "         --format, default NV12,            optional, fourcc of format\n"
        "         --count,  default -1,              optional, how many frames to capture, default unlimited\n"
        "         --device,                          required, path of video device\n"
        "         --memtype,                         optional, 0:v4l2 fd, 1:drm fd, default 0\n"
        "         --vop,                             optional, frame render out\n"
        "         --stream-to,                       optional, output file path, if <file> is '-', then the data is written to stdout\n"
        "         --stream-count, default 60         optional, how many frames to write files\n"
        "         --stream-skip, default 30          optional, how many frames to skip befor writing file\n"
        "         --silent,                          optional, subpress debug log\n"
        "         --hdr <val>,                       optional, hdr mode, val 2 means hdrx2, 3 means hdrx3 \n"
        "         --sync-to-raw,                     optional, write yuv files in sync with raw\n"
        "         --limit,                           optional, yuv limit range\n"
        ,argv[0]);
    exit(-1);
}

int main(int argc, char **argv)
{
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    sigaddset(&mask, SIGQUIT);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

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
        .buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
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
        .yuv_dir_path = {'\0'},
        ._is_yuv_dir_exist = 0,
        .capture_yuv_num = 0,
        .is_capture_yuv = 0,
    };
    parse_args(argc, argv, &main_ctx);
    v4l2_routine(&main_ctx);
    g_main_ctx = &main_ctx;
    pthread_sigmask(SIG_UNBLOCK, &mask, NULL);
    mainloop(&main_ctx);
    v4l2_deinit(&main_ctx);

    return 0;
}
#else
#include "video_capture.cpp"
#endif
