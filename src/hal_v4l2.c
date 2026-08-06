/*
 * hal_v4l2.c -- standalone V4L2 DMA-BUF capture to OpenIMP AVC adapter
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "hal_internal.h"

/* One buffer may be owned by AVPU while the ISP fills the other.  A third
 * full-resolution NV12 allocation adds 5.5 MiB of contiguous pressure on T41
 * without increasing this single-in-flight encoder's throughput. */
#define RSS_V4L2_BUFFER_COUNT 2U
#define OPENIMP_AVC_PIXFMT_NV12 10U

typedef struct OpenIMPAVCEncoder OpenIMPAVCEncoder;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t fps_num;
    uint32_t fps_den;
    uint32_t bitrate;
    uint32_t gop_length;
    uint32_t stream_buffer_count;
    uint32_t stream_buffer_size;
    uint8_t profile;
    uint8_t rate_control;
    uint8_t initial_qp;
    uint8_t min_qp;
    uint8_t max_qp;
    uint8_t entropy_coding;
} OpenIMPAVCConfig;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t size;
    uint32_t physical_address;
    uintptr_t virtual_address;
    uint64_t timestamp;
    void *cookie;
} OpenIMPAVCFrame;

typedef struct {
    const uint8_t *data;
    uint32_t length;
    uint32_t physical_address;
    uint64_t timestamp;
    int keyframe;
    void *cookie;
    void *private_stream;
    void *private_user;
} OpenIMPAVCPacket;

extern int OpenIMP_AVC_Create(OpenIMPAVCEncoder **encoder,
                              const OpenIMPAVCConfig *config) __attribute__((weak));
extern int OpenIMP_AVC_Destroy(OpenIMPAVCEncoder *encoder) __attribute__((weak));
extern int OpenIMP_AVC_Submit(OpenIMPAVCEncoder *encoder,
                              const OpenIMPAVCFrame *frame) __attribute__((weak));
extern int OpenIMP_AVC_Dequeue(OpenIMPAVCEncoder *encoder,
                               OpenIMPAVCPacket *packet,
                               uint32_t timeout_ms) __attribute__((weak));
extern int OpenIMP_AVC_Release(OpenIMPAVCEncoder *encoder,
                               OpenIMPAVCPacket *packet) __attribute__((weak));
extern int OpenIMP_AVC_RequestIDR(OpenIMPAVCEncoder *encoder) __attribute__((weak));
extern int OpenIMP_AVC_ImportDMABuf(int dma_buf_fd, uint32_t size,
                                    uint32_t *physical_address) __attribute__((weak));

struct rss_v4l2_buffer {
    void *address;
    size_t length;
    int dma_fd;
    uint32_t physical_address;
};

struct rss_v4l2_h264 {
    int video_fd;
    enum v4l2_buf_type type;
    struct rss_v4l2_buffer buffers[RSS_V4L2_BUFFER_COUNT];
    uint32_t buffer_count;
    uint32_t width;
    uint32_t height;
    uint32_t frame_size;
    uint32_t sequence;
    uint32_t dequeued_index;
    OpenIMPAVCEncoder *encoder;
    OpenIMPAVCPacket packet;
    rss_nal_unit_t nal;
    int streaming;
    int source_dequeued;
    int packet_valid;
};

static int v4l2_ioctl(int fd, unsigned long request, void *argument)
{
    int ret;

    do {
        ret = ioctl(fd, request, argument);
    } while (ret < 0 && errno == EINTR);
    return ret < 0 ? -errno : 0;
}

static uint8_t openimp_profile(int profile)
{
    if (profile == 0)
        return 66;
    if (profile == 1)
        return 77;
    return 100;
}

static int openimp_symbols_available(void)
{
    return OpenIMP_AVC_Create && OpenIMP_AVC_Destroy &&
           OpenIMP_AVC_Submit && OpenIMP_AVC_Dequeue &&
           OpenIMP_AVC_Release && OpenIMP_AVC_RequestIDR &&
           OpenIMP_AVC_ImportDMABuf;
}

static int queue_buffer(rss_v4l2_h264_t *backend, uint32_t index)
{
    struct v4l2_buffer buffer;

    memset(&buffer, 0, sizeof(buffer));
    buffer.type = backend->type;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    return v4l2_ioctl(backend->video_fd, VIDIOC_QBUF, &buffer);
}

static int release_pending(rss_v4l2_h264_t *backend, int requeue)
{
    int ret = 0;

    if (!backend->source_dequeued)
        return 0;
    if (!backend->packet_valid) {
        ret = OpenIMP_AVC_Dequeue(backend->encoder, &backend->packet, 2000);
        if (ret)
            return ret;
        backend->packet_valid = 1;
    }
    ret = OpenIMP_AVC_Release(backend->encoder, &backend->packet);
    if (!ret && requeue)
        ret = queue_buffer(backend, backend->dequeued_index);
    backend->packet_valid = 0;
    backend->source_dequeued = 0;
    return ret;
}

int rss_v4l2_h264_create(rss_v4l2_h264_t **backend_out, const char *video_device,
                         const rss_video_config_t *config)
{
    rss_v4l2_h264_t *backend;
    struct v4l2_requestbuffers request;
    struct v4l2_format format;
    uint32_t index;
    int ret = -EINVAL;

    if (!backend_out || !config || config->codec != RSS_CODEC_H264 ||
        !config->width || !config->height || !config->fps_num ||
        !config->fps_den || !config->bitrate)
        return -EINVAL;
    *backend_out = NULL;
    if (!openimp_symbols_available())
        return -ENOTSUP;

    backend = calloc(1, sizeof(*backend));
    if (!backend)
        return -ENOMEM;
    backend->video_fd = -1;
    backend->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (index = 0; index < RSS_V4L2_BUFFER_COUNT; ++index)
        backend->buffers[index].dma_fd = -1;

    backend->video_fd = open(video_device ? video_device : "/dev/video0",
                             O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (backend->video_fd < 0) {
        ret = -errno;
        goto fail;
    }
    memset(&format, 0, sizeof(format));
    format.type = backend->type;
    format.fmt.pix.width = config->width;
    format.fmt.pix.height = config->height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    ret = v4l2_ioctl(backend->video_fd, VIDIOC_S_FMT, &format);
    if (ret)
        goto fail;
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_NV12 ||
        format.fmt.pix.width != config->width ||
        format.fmt.pix.height != config->height) {
        ret = -EINVAL;
        goto fail;
    }
    backend->width = format.fmt.pix.width;
    backend->height = format.fmt.pix.height;
    backend->frame_size = format.fmt.pix.sizeimage;

    memset(&request, 0, sizeof(request));
    request.count = RSS_V4L2_BUFFER_COUNT;
    request.type = backend->type;
    request.memory = V4L2_MEMORY_MMAP;
    ret = v4l2_ioctl(backend->video_fd, VIDIOC_REQBUFS, &request);
    if (ret)
        goto fail;
    if (request.count < RSS_V4L2_BUFFER_COUNT) {
        ret = -ENOMEM;
        goto fail;
    }
    backend->buffer_count = RSS_V4L2_BUFFER_COUNT;

    for (index = 0; index < backend->buffer_count; ++index) {
        struct v4l2_exportbuffer export;
        struct v4l2_buffer buffer;

        memset(&buffer, 0, sizeof(buffer));
        buffer.type = backend->type;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        ret = v4l2_ioctl(backend->video_fd, VIDIOC_QUERYBUF, &buffer);
        if (ret)
            goto fail;
        memset(&export, 0, sizeof(export));
        export.type = backend->type;
        export.index = index;
        export.flags = O_CLOEXEC;
        ret = v4l2_ioctl(backend->video_fd, VIDIOC_EXPBUF, &export);
        if (ret)
            goto fail;
        backend->buffers[index].dma_fd = export.fd;
        backend->buffers[index].length = buffer.length;
        backend->buffers[index].address = mmap(NULL, buffer.length,
                                               PROT_READ, MAP_SHARED,
                                               export.fd, 0);
        if (backend->buffers[index].address == MAP_FAILED) {
            backend->buffers[index].address = NULL;
            ret = -errno;
            goto fail;
        }
        ret = OpenIMP_AVC_ImportDMABuf(
            export.fd, buffer.length,
            &backend->buffers[index].physical_address);
        if (ret)
            goto fail;
    }

    {
        uint8_t min_qp = config->min_qp > 0 ? config->min_qp : 15;
        uint8_t max_qp = config->max_qp > 0 ? config->max_qp : 45;
        uint8_t initial_qp = config->init_qp > 0 ? config->init_qp : 26;

        if (initial_qp < min_qp)
            initial_qp = min_qp;
        if (initial_qp > max_qp)
            initial_qp = max_qp;
        OpenIMPAVCConfig avc = {
            .width = config->width,
            .height = config->height,
            .fps_num = config->fps_num,
            .fps_den = config->fps_den,
            .bitrate = config->bitrate,
            .gop_length = config->gop_length ? config->gop_length : config->fps_num,
            .stream_buffer_count = config->max_stream_cnt ? config->max_stream_cnt : 4,
            .stream_buffer_size = config->stream_buf_size,
            .profile = openimp_profile(config->profile),
            .rate_control = config->rc_mode <= RSS_RC_VBR ? config->rc_mode : RSS_RC_CBR,
            .initial_qp = initial_qp,
            .min_qp = min_qp,
            .max_qp = max_qp,
            .entropy_coding = config->profile != 0,
        };

        ret = OpenIMP_AVC_Create(&backend->encoder, &avc);
        if (ret)
            goto fail;
    }
    HAL_LOG_INFO("V4L2 AVC backend ready: %s %ux%u %u/%u",
                 video_device ? video_device : "/dev/video0", backend->width,
                 backend->height, config->fps_num, config->fps_den);
    *backend_out = backend;
    return 0;

fail:
    rss_v4l2_h264_destroy(backend);
    return ret;
}

void rss_v4l2_h264_destroy(rss_v4l2_h264_t *backend)
{
    uint32_t index;

    if (!backend)
        return;
    rss_v4l2_h264_stop(backend);
    if (backend->encoder)
        OpenIMP_AVC_Destroy(backend->encoder);
    for (index = 0; index < RSS_V4L2_BUFFER_COUNT; ++index) {
        if (backend->buffers[index].address)
            munmap(backend->buffers[index].address,
                   backend->buffers[index].length);
        if (backend->buffers[index].dma_fd >= 0)
            close(backend->buffers[index].dma_fd);
    }
    if (backend->video_fd >= 0) {
        struct v4l2_requestbuffers request;

        memset(&request, 0, sizeof(request));
        request.type = backend->type;
        request.memory = V4L2_MEMORY_MMAP;
        v4l2_ioctl(backend->video_fd, VIDIOC_REQBUFS, &request);
        close(backend->video_fd);
    }
    free(backend);
}

int rss_v4l2_h264_start(rss_v4l2_h264_t *backend)
{
    uint32_t index;
    int ret;

    if (!backend)
        return -EINVAL;
    if (backend->streaming)
        return 0;
    for (index = 0; index < backend->buffer_count; ++index) {
        ret = queue_buffer(backend, index);
        if (ret)
            return ret;
    }
    ret = v4l2_ioctl(backend->video_fd, VIDIOC_STREAMON, &backend->type);
    if (ret)
        return ret;
    backend->streaming = 1;
    return 0;
}

int rss_v4l2_h264_stop(rss_v4l2_h264_t *backend)
{
    int ret = 0;
    int stream_ret;

    if (!backend)
        return -EINVAL;
    if (backend->source_dequeued)
        ret = release_pending(backend, 0);
    if (!backend->streaming)
        return ret;
    stream_ret = v4l2_ioctl(backend->video_fd, VIDIOC_STREAMOFF,
                            &backend->type);
    backend->streaming = 0;
    return ret ? ret : stream_ret;
}

int rss_v4l2_h264_poll(rss_v4l2_h264_t *backend, uint32_t timeout_ms)
{
    struct v4l2_buffer buffer;
    struct pollfd poll_fd;
    OpenIMPAVCFrame frame;
    int ret;

    if (!backend || !backend->streaming)
        return -EINVAL;
    if (backend->packet_valid)
        return 0;
    if (backend->source_dequeued) {
        ret = OpenIMP_AVC_Dequeue(backend->encoder, &backend->packet,
                                  timeout_ms);
        if (!ret)
            backend->packet_valid = 1;
        return ret;
    }
    poll_fd.fd = backend->video_fd;
    poll_fd.events = POLLIN;
    poll_fd.revents = 0;
    do {
        ret = poll(&poll_fd, 1, (int)timeout_ms);
    } while (ret < 0 && errno == EINTR);
    if (ret == 0)
        return -ETIMEDOUT;
    if (ret < 0)
        return -errno;
    if (!(poll_fd.revents & POLLIN))
        return -EIO;

    memset(&buffer, 0, sizeof(buffer));
    buffer.type = backend->type;
    buffer.memory = V4L2_MEMORY_MMAP;
    ret = v4l2_ioctl(backend->video_fd, VIDIOC_DQBUF, &buffer);
    if (ret)
        return ret;
    if (buffer.index >= backend->buffer_count)
        return -EIO;
    backend->source_dequeued = 1;
    backend->dequeued_index = buffer.index;
    backend->sequence = buffer.sequence;

    memset(&frame, 0, sizeof(frame));
    frame.width = backend->width;
    frame.height = backend->height;
    frame.pixel_format = OPENIMP_AVC_PIXFMT_NV12;
    frame.size = buffer.bytesused ? buffer.bytesused : backend->frame_size;
    frame.physical_address = backend->buffers[buffer.index].physical_address;
    frame.virtual_address = (uintptr_t)backend->buffers[buffer.index].address;
    frame.timestamp = (uint64_t)buffer.timestamp.tv_sec * 1000000U +
                      (uint64_t)buffer.timestamp.tv_usec;
    frame.cookie = (void *)(uintptr_t)buffer.index;
    ret = OpenIMP_AVC_Submit(backend->encoder, &frame);
    if (ret) {
        queue_buffer(backend, buffer.index);
        backend->source_dequeued = 0;
        return ret;
    }
    ret = OpenIMP_AVC_Dequeue(backend->encoder, &backend->packet, timeout_ms);
    if (!ret)
        backend->packet_valid = 1;
    return ret;
}

int rss_v4l2_h264_get_frame(rss_v4l2_h264_t *backend, rss_frame_t *frame)
{
    if (!backend || !frame || !backend->packet_valid)
        return -EINVAL;
    memset(frame, 0, sizeof(*frame));
    backend->nal.data = backend->packet.data;
    backend->nal.length = backend->packet.length;
    backend->nal.type = backend->packet.keyframe ? RSS_NAL_H264_IDR : RSS_NAL_H264_SLICE;
    backend->nal.frame_end = true;
    frame->nals = &backend->nal;
    frame->nal_count = 1;
    frame->codec = RSS_CODEC_H264;
    frame->timestamp = backend->packet.timestamp;
    frame->seq = backend->sequence;
    frame->is_key = backend->packet.keyframe != 0;
    frame->_priv = backend;
    return 0;
}

int rss_v4l2_h264_release_frame(rss_v4l2_h264_t *backend, rss_frame_t *frame)
{
    int ret;

    if (!backend || !frame || frame->_priv != backend)
        return -EINVAL;
    ret = release_pending(backend, backend->streaming);
    frame->_priv = NULL;
    frame->nals = NULL;
    frame->nal_count = 0;
    return ret;
}

int rss_v4l2_h264_request_idr(rss_v4l2_h264_t *backend)
{
    if (!backend)
        return -EINVAL;
    return OpenIMP_AVC_RequestIDR(backend->encoder);
}
