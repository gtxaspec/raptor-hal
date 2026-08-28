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
#include <limits.h>
#include <poll.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "hal_internal.h"

/* One buffer may be owned by AVPU while the ISP fills the other.  A third
 * full-resolution NV12 allocation adds 5.5 MiB of contiguous pressure on T41
 * without increasing this single-in-flight encoder's throughput. */
#define RSS_V4L2_BUFFER_COUNT 2U
#define OPENIMP_AVC_PIXFMT_NV12 10U

/*
 * OpenIMP is built after raptor-hal in Thingino, so its installed public
 * header is not available here without introducing a package dependency
 * cycle. These declarations are copied verbatim from:
 *
 *   opensensor/openimp@e236efffed2b2399e9d04dc159ece14a345b6e69
 *   include/openimp/openimp_avc.h
 *
 * That commit is the bridge ABI contract. Update this block and the pinned
 * OpenIMP firmware revision together if the public structures ever change.
 */
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

extern int OpenIMP_AVC_Create(OpenIMPAVCEncoder **encoder, const OpenIMPAVCConfig *config)
    __attribute__((weak));
extern int OpenIMP_AVC_Destroy(OpenIMPAVCEncoder *encoder) __attribute__((weak));
extern int OpenIMP_AVC_Submit(OpenIMPAVCEncoder *encoder, const OpenIMPAVCFrame *frame)
    __attribute__((weak));
extern int OpenIMP_AVC_Dequeue(OpenIMPAVCEncoder *encoder, OpenIMPAVCPacket *packet,
                               uint32_t timeout_ms) __attribute__((weak));
extern int OpenIMP_AVC_Release(OpenIMPAVCEncoder *encoder, OpenIMPAVCPacket *packet)
    __attribute__((weak));
extern int OpenIMP_AVC_RequestIDR(OpenIMPAVCEncoder *encoder) __attribute__((weak));
extern int OpenIMP_AVC_SetBitrate(OpenIMPAVCEncoder *encoder, uint32_t bitrate)
    __attribute__((weak));
extern int OpenIMP_AVC_SetGopLength(OpenIMPAVCEncoder *encoder, uint32_t gop_length)
    __attribute__((weak));
extern int OpenIMP_AVC_ImportDMABuf(int dma_buf_fd, uint32_t size, uint32_t *physical_address)
    __attribute__((weak));

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
    int source_requeue_pending;
    int packet_valid;
    int warned_key_mismatch;
    atomic_uint pending_bitrate;
    atomic_uint target_bitrate;
    atomic_uint pending_gop;
    atomic_uint target_gop;
    atomic_uint average_bitrate;
    uint64_t bitrate_window_start;
    uint64_t bitrate_window_bits;
};

static uint64_t monotonic_ms(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return 0;
    return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
}

static uint32_t remaining_timeout_ms(uint64_t deadline_ms)
{
    uint64_t now_ms = monotonic_ms();
    uint64_t remaining;

    if (now_ms >= deadline_ms)
        return 0;
    remaining = deadline_ms - now_ms;
    return remaining > UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}

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
    return OpenIMP_AVC_Create && OpenIMP_AVC_Destroy && OpenIMP_AVC_Submit && OpenIMP_AVC_Dequeue &&
           OpenIMP_AVC_Release && OpenIMP_AVC_RequestIDR && OpenIMP_AVC_ImportDMABuf;
}

static int apply_pending_bitrate(rss_v4l2_h264_t *backend)
{
    unsigned int bitrate = atomic_exchange(&backend->pending_bitrate, 0);

    if (!bitrate)
        return 0;
    if (!OpenIMP_AVC_SetBitrate)
        return -ENOTSUP;
    if (OpenIMP_AVC_SetBitrate(backend->encoder, bitrate) != 0) {
        unsigned int empty = 0;

        atomic_compare_exchange_strong(&backend->pending_bitrate, &empty, bitrate);
        return -EIO;
    }
    return 0;
}

static int apply_pending_gop(rss_v4l2_h264_t *backend)
{
    unsigned int gop = atomic_exchange(&backend->pending_gop, 0);

    if (!gop)
        return 0;
    if (!OpenIMP_AVC_SetGopLength)
        return -ENOTSUP;
    if (OpenIMP_AVC_SetGopLength(backend->encoder, gop) != 0) {
        unsigned int empty = 0;

        atomic_compare_exchange_strong(&backend->pending_gop, &empty, gop);
        return -EIO;
    }
    return 0;
}

static void update_average_bitrate(rss_v4l2_h264_t *backend)
{
    uint64_t timestamp = backend->packet.timestamp;
    uint64_t elapsed;
    uint64_t bitrate;

    if (!backend->bitrate_window_start || timestamp <= backend->bitrate_window_start) {
        backend->bitrate_window_start = timestamp;
        backend->bitrate_window_bits = 0;
        return;
    }
    backend->bitrate_window_bits += (uint64_t)backend->packet.length * 8U;
    elapsed = timestamp - backend->bitrate_window_start;
    if (elapsed < 1000000U)
        return;
    bitrate = backend->bitrate_window_bits * 1000000U / elapsed;
    atomic_store(&backend->average_bitrate,
                 bitrate > UINT32_MAX ? UINT32_MAX : (unsigned int)bitrate);
    backend->bitrate_window_start = timestamp;
    backend->bitrate_window_bits = 0;
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

static int requeue_source(rss_v4l2_h264_t *backend)
{
    int ret = queue_buffer(backend, backend->dequeued_index);

    if (ret) {
        backend->source_requeue_pending = 1;
        return ret;
    }
    backend->source_requeue_pending = 0;
    backend->source_dequeued = 0;
    return 0;
}

static int release_pending(rss_v4l2_h264_t *backend, int requeue)
{
    int ret;

    if (!backend->source_dequeued)
        return 0;
    if (backend->source_requeue_pending) {
        if (requeue)
            return requeue_source(backend);
        backend->source_requeue_pending = 0;
        backend->source_dequeued = 0;
        return 0;
    }
    if (!backend->packet_valid) {
        ret = OpenIMP_AVC_Dequeue(backend->encoder, &backend->packet, 2000);
        if (ret)
            return ret;
        backend->packet_valid = 1;
    }
    ret = OpenIMP_AVC_Release(backend->encoder, &backend->packet);
    /* The pinned OpenIMP ABI consumes and clears the packet even when its
     * underlying release reports an error. Do not retry a dead packet; retain
     * the V4L2 index so the next poll can recover by requeueing it. */
    backend->packet_valid = 0;
    if (ret) {
        backend->source_requeue_pending = requeue;
        return ret;
    }
    if (requeue)
        return requeue_source(backend);
    backend->source_dequeued = 0;
    return 0;
}

/*
 * The bridge contract is one complete H.264 access unit in Annex-B form.
 * RSD's SPS/PPS cache and RMR's muxer both parse start codes, so accepting
 * length-prefixed output here would fail downstream even if capture itself
 * appeared healthy. Derive keyframe state from the IDR NAL as well: this is
 * the value RVD publishes into the ring and RSD uses to release new clients.
 */
static int classify_h264_annexb(const uint8_t *data, uint32_t length, int *is_key)
{
    uint32_t offset;
    int saw_start_code = 0;
    int saw_vcl = 0;

    if (!data || !length || !is_key)
        return -EINVAL;
    *is_key = 0;
    for (offset = 0; offset + 3U < length; ++offset) {
        uint32_t nal_offset;
        uint8_t nal_type;

        if (data[offset] != 0 || data[offset + 1U] != 0)
            continue;
        if (data[offset + 2U] == 1U)
            nal_offset = offset + 3U;
        else if (offset + 4U < length && data[offset + 2U] == 0 && data[offset + 3U] == 1U)
            nal_offset = offset + 4U;
        else
            continue;
        if (nal_offset >= length)
            continue;
        saw_start_code = 1;
        nal_type = data[nal_offset] & 0x1fU;
        if (nal_type >= 1U && nal_type <= 5U)
            saw_vcl = 1;
        if (nal_type == 5U)
            *is_key = 1;
        offset = nal_offset;
    }
    return saw_start_code && saw_vcl ? 0 : -EPROTO;
}

int rss_v4l2_h264_create(rss_v4l2_h264_t **backend_out, const char *video_device,
                         const rss_video_config_t *config)
{
    rss_v4l2_h264_t *backend;
    struct v4l2_requestbuffers request;
    struct v4l2_format format;
    uint32_t index;
    int ret = -EINVAL;

    if (!backend_out || !config || config->codec != RSS_CODEC_H264 || !config->width ||
        !config->height || !config->fps_num || !config->fps_den || !config->bitrate)
        return -EINVAL;
    *backend_out = NULL;
    if (!openimp_symbols_available())
        return -ENOTSUP;

    backend = calloc(1, sizeof(*backend));
    if (!backend)
        return -ENOMEM;
    backend->video_fd = -1;
    backend->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    atomic_init(&backend->pending_bitrate, 0);
    atomic_init(&backend->target_bitrate, config->bitrate);
    atomic_init(&backend->pending_gop, 0);
    atomic_init(&backend->target_gop, config->gop_length);
    atomic_init(&backend->average_bitrate, 0);
    for (index = 0; index < RSS_V4L2_BUFFER_COUNT; ++index)
        backend->buffers[index].dma_fd = -1;

    backend->video_fd =
        open(video_device ? video_device : "/dev/video0", O_RDWR | O_NONBLOCK | O_CLOEXEC);
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
    if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_NV12 || format.fmt.pix.width != config->width ||
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
        backend->buffers[index].address =
            mmap(NULL, buffer.length, PROT_READ, MAP_SHARED, export.fd, 0);
        if (backend->buffers[index].address == MAP_FAILED) {
            backend->buffers[index].address = NULL;
            ret = -errno;
            goto fail;
        }
        ret = OpenIMP_AVC_ImportDMABuf(export.fd, buffer.length,
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
                 video_device ? video_device : "/dev/video0", backend->width, backend->height,
                 config->fps_num, config->fps_den);
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
            munmap(backend->buffers[index].address, backend->buffers[index].length);
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
    stream_ret = v4l2_ioctl(backend->video_fd, VIDIOC_STREAMOFF, &backend->type);
    backend->streaming = 0;
    return ret ? ret : stream_ret;
}

int rss_v4l2_h264_poll(rss_v4l2_h264_t *backend, uint32_t timeout_ms)
{
    struct v4l2_buffer buffer;
    struct pollfd poll_fd;
    OpenIMPAVCFrame frame;
    uint64_t deadline_ms;
    uint32_t remaining_ms;
    int ret;

    if (!backend || !backend->streaming)
        return -EINVAL;
    deadline_ms = monotonic_ms() + timeout_ms;
    if (backend->packet_valid)
        return 0;
    if (backend->source_requeue_pending) {
        ret = requeue_source(backend);
        if (ret)
            return ret;
    }
    if (backend->source_dequeued) {
        ret = OpenIMP_AVC_Dequeue(backend->encoder, &backend->packet,
                                  remaining_timeout_ms(deadline_ms));
        if (!ret)
            backend->packet_valid = 1;
        return ret;
    }
    ret = apply_pending_bitrate(backend);
    if (ret)
        return ret;
    ret = apply_pending_gop(backend);
    if (ret)
        return ret;
    poll_fd.fd = backend->video_fd;
    poll_fd.events = POLLIN;
    poll_fd.revents = 0;
    do {
        remaining_ms = remaining_timeout_ms(deadline_ms);
        ret = poll(&poll_fd, 1, remaining_ms > INT_MAX ? INT_MAX : (int)remaining_ms);
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
    frame.timestamp =
        (uint64_t)buffer.timestamp.tv_sec * 1000000U + (uint64_t)buffer.timestamp.tv_usec;
    frame.cookie = (void *)(uintptr_t)buffer.index;
    ret = OpenIMP_AVC_Submit(backend->encoder, &frame);
    if (ret) {
        int queue_ret = requeue_source(backend);

        if (queue_ret)
            return queue_ret;
        return ret;
    }
    ret =
        OpenIMP_AVC_Dequeue(backend->encoder, &backend->packet, remaining_timeout_ms(deadline_ms));
    if (!ret)
        backend->packet_valid = 1;
    return ret;
}

int rss_v4l2_h264_get_frame(rss_v4l2_h264_t *backend, rss_frame_t *frame)
{
    int is_key;
    int ret;

    if (!backend || !frame || !backend->packet_valid)
        return -EINVAL;
    ret = classify_h264_annexb(backend->packet.data, backend->packet.length, &is_key);
    if (ret) {
        int release_ret;

        HAL_LOG_ERR("OpenIMP packet is not a complete Annex-B H.264 access unit");
        release_ret = release_pending(backend, backend->streaming);
        if (release_ret)
            HAL_LOG_ERR("failed to release invalid OpenIMP packet: %d", release_ret);
        return ret;
    }
    if (!!backend->packet.keyframe != !!is_key && !backend->warned_key_mismatch) {
        HAL_LOG_WARN("OpenIMP keyframe flag disagrees with Annex-B NALs; "
                     "using NAL classification");
        backend->warned_key_mismatch = 1;
    }
    memset(frame, 0, sizeof(*frame));
    backend->nal.data = backend->packet.data;
    backend->nal.length = backend->packet.length;
    backend->nal.type = is_key ? RSS_NAL_H264_IDR : RSS_NAL_H264_SLICE;
    backend->nal.frame_end = true;
    frame->nals = &backend->nal;
    frame->nal_count = 1;
    frame->codec = RSS_CODEC_H264;
    frame->timestamp = backend->packet.timestamp;
    frame->seq = backend->sequence;
    frame->is_key = is_key != 0;
    frame->_priv = backend;
    update_average_bitrate(backend);
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

/* Thread contract: called from RVD's ctrl thread while the encoder
 * thread is concurrently in poll/dequeue on the same handle. No
 * userspace lock on purpose: serialization is the AL codec command
 * path's job, the same interlock the vendor SDK's RequestIDR has
 * relied on in production for years. */
int rss_v4l2_h264_request_idr(rss_v4l2_h264_t *backend)
{
    if (!backend)
        return -EINVAL;
    return OpenIMP_AVC_RequestIDR(backend->encoder);
}

int rss_v4l2_h264_set_bitrate(rss_v4l2_h264_t *backend, uint32_t bitrate)
{
    if (!backend || !bitrate)
        return -EINVAL;
    if (!OpenIMP_AVC_SetBitrate)
        return -ENOTSUP;
    atomic_store(&backend->target_bitrate, bitrate);
    atomic_store(&backend->pending_bitrate, bitrate);
    return 0;
}

int rss_v4l2_h264_get_bitrate(rss_v4l2_h264_t *backend, uint32_t *target_bitrate,
                              uint32_t *average_bitrate)
{
    if (!backend || (!target_bitrate && !average_bitrate))
        return -EINVAL;
    if (target_bitrate)
        *target_bitrate = atomic_load(&backend->target_bitrate);
    if (average_bitrate)
        *average_bitrate = atomic_load(&backend->average_bitrate);
    return 0;
}

int rss_v4l2_h264_set_gop(rss_v4l2_h264_t *backend, uint32_t gop_length)
{
    if (!backend || !gop_length)
        return -EINVAL;
    if (!OpenIMP_AVC_SetGopLength)
        return -ENOTSUP;
    atomic_store(&backend->target_gop, gop_length);
    atomic_store(&backend->pending_gop, gop_length);
    return 0;
}

int rss_v4l2_h264_get_gop(rss_v4l2_h264_t *backend, uint32_t *gop_length)
{
    if (!backend || !gop_length)
        return -EINVAL;
    *gop_length = atomic_load(&backend->target_gop);
    return 0;
}

/* ================================================================
 * The "v4l2" backend ops table
 *
 * Composed from the IMP table: ISP, sensor and system ops are
 * inherited verbatim (on an OpenIMP system libimp IS OpenIMP, the
 * same ABI the tuning path already talks), the encoder slots mount
 * this file's capture/encode queue, and the subsystems this backend
 * does not have (framesource graph, OSD, IVS, JPEG, the IMP encoder
 * feature surface) are NULL so RSS_HAL_CALL answers RSS_ERR_NOTSUP
 * without any caller-side allowlist.
 * ================================================================ */

static int v4l2_ops_enc_create_channel(void *vctx, int chn, const rss_video_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)vctx;

    if (chn != 0)
        return RSS_ERR_NOTSUP; /* single H.264 channel */
    if (c->v4l2)
        return -EBUSY;
    return rss_v4l2_h264_create(&c->v4l2, c->v4l2_device[0] ? c->v4l2_device : "/dev/video0", cfg);
}

static int v4l2_ops_enc_destroy_channel(void *vctx, int chn)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)vctx;

    if (chn != 0 || !c->v4l2)
        return -EINVAL;
    rss_v4l2_h264_destroy(c->v4l2);
    c->v4l2 = NULL;
    return 0;
}

#define V4L2_OPS_WRAP(name, call)                                                                  \
    static int v4l2_ops_##name(void *vctx, int chn)                                                \
    {                                                                                              \
        rss_hal_ctx_t *c = (rss_hal_ctx_t *)vctx;                                                  \
        if (chn != 0 || !c->v4l2)                                                                  \
            return -EINVAL;                                                                        \
        return call(c->v4l2);                                                                      \
    }

V4L2_OPS_WRAP(enc_start, rss_v4l2_h264_start)
V4L2_OPS_WRAP(enc_stop, rss_v4l2_h264_stop)
V4L2_OPS_WRAP(enc_request_idr, rss_v4l2_h264_request_idr)

static int v4l2_ops_enc_poll(void *vctx, int chn, uint32_t timeout_ms)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)vctx;

    if (chn != 0 || !c->v4l2)
        return -EINVAL;
    return rss_v4l2_h264_poll(c->v4l2, timeout_ms);
}

static int v4l2_ops_enc_get_frame(void *vctx, int chn, rss_frame_t *frame)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)vctx;

    if (chn != 0 || !c->v4l2)
        return -EINVAL;
    return rss_v4l2_h264_get_frame(c->v4l2, frame);
}

static int v4l2_ops_enc_release_frame(void *vctx, int chn, rss_frame_t *frame)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)vctx;

    if (chn != 0 || !c->v4l2)
        return -EINVAL;
    return rss_v4l2_h264_release_frame(c->v4l2, frame);
}

/* deinit: the encoder instance goes first, then the inherited IMP
 * teardown releases the sensor and ISP it brought up in init. */
static int v4l2_ops_deinit(void *vctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)vctx;

    if (c->v4l2) {
        rss_v4l2_h264_destroy(c->v4l2);
        c->v4l2 = NULL;
    }
    return hal_imp_ops()->deinit(vctx);
}

void rss_hal_v4l2_set_device(rss_hal_ctx_t *ctx, const char *device)
{
    if (ctx && device)
        snprintf(ctx->v4l2_device, sizeof(ctx->v4l2_device), "%s", device);
}

const rss_hal_ops_t *hal_v4l2_backend_ops(void)
{
    static rss_hal_ops_t table;
    static bool built;
    rss_hal_ops_t *ops = &table;

    if (built)
        return &table;

    table = *hal_imp_ops();

    /* Absent subsystems answer RSS_ERR_NOTSUP via the NULL-slot rule. */
    ops->bind = NULL;
    ops->enc_create_group = NULL;
    ops->enc_destroy_group = NULL;
    ops->enc_flush_stream = NULL;
    ops->enc_get_avg_bitrate = NULL;
    ops->enc_get_channel_attr = NULL;
    ops->enc_get_chn_ave_bitrate = NULL;
    ops->enc_get_chn_enc_type = NULL;
    ops->enc_get_chn_gop_attr = NULL;
    ops->enc_get_color2grey = NULL;
    ops->enc_get_crop = NULL;
    ops->enc_get_denoise = NULL;
    ops->enc_get_eval_info = NULL;
    ops->enc_get_fd = NULL;
    ops->enc_get_fps = NULL;
    ops->enc_get_gdr = NULL;
    ops->enc_get_gop_attr = NULL;
    ops->enc_get_gop_mode = NULL;
    ops->enc_get_h264_trans = NULL;
    ops->enc_get_h264_vui = NULL;
    ops->enc_get_h265_trans = NULL;
    ops->enc_get_h265_vui = NULL;
    ops->enc_get_jpeg_ql = NULL;
    ops->enc_get_jpeg_qp = NULL;
    ops->enc_get_max_same_scene_cnt = NULL;
    ops->enc_get_max_stream_cnt = NULL;
    ops->enc_get_mbrc = NULL;
    ops->enc_get_pool = NULL;
    ops->enc_get_pskip = NULL;
    ops->enc_get_qpg_mode = NULL;
    ops->enc_get_rc_options = NULL;
    ops->enc_get_rmem_info = NULL;
    ops->enc_get_roi = NULL;
    ops->enc_get_srd = NULL;
    ops->enc_get_stream_buf_size = NULL;
    ops->enc_get_super_frame = NULL;
    ops->enc_inject_stream_shm = NULL;
    ops->enc_insert_userdata = NULL;
    ops->enc_poll_module_stream = NULL;
    ops->enc_query = NULL;
    ops->enc_register_channel = NULL;
    ops->enc_request_gdr = NULL;
    ops->enc_request_pskip = NULL;
    ops->enc_set_bitrate = NULL;
    ops->enc_set_bufshare = NULL;
    ops->enc_set_chn_entropy_mode = NULL;
    ops->enc_set_chn_gop_attr = NULL;
    ops->enc_set_color2grey = NULL;
    ops->enc_set_crop = NULL;
    ops->enc_set_denoise = NULL;
    ops->enc_set_fps = NULL;
    ops->enc_set_gdr = NULL;
    ops->enc_set_gop = NULL;
    ops->enc_set_gop_attr = NULL;
    ops->enc_set_gop_mode = NULL;
    ops->enc_set_h264_trans = NULL;
    ops->enc_set_h264_vui = NULL;
    ops->enc_set_h265_trans = NULL;
    ops->enc_set_h265_vui = NULL;
    ops->enc_set_jpeg_ql = NULL;
    ops->enc_set_jpeg_qp = NULL;
    ops->enc_set_map_roi = NULL;
    ops->enc_set_max_pic_size = NULL;
    ops->enc_set_max_psnr = NULL;
    ops->enc_set_max_same_scene_cnt = NULL;
    ops->enc_set_max_stream_cnt = NULL;
    ops->enc_set_mbrc = NULL;
    ops->enc_set_pool = NULL;
    ops->enc_set_pskip = NULL;
    ops->enc_set_qp = NULL;
    ops->enc_set_qp_bounds = NULL;
    ops->enc_set_qp_bounds_per_frame = NULL;
    ops->enc_set_qp_ip_delta = NULL;
    ops->enc_set_qp_pb_delta = NULL;
    ops->enc_set_qpg_ai = NULL;
    ops->enc_set_qpg_mode = NULL;
    ops->enc_set_rc_mode = NULL;
    ops->enc_set_rc_options = NULL;
    ops->enc_set_resize_mode = NULL;
    ops->enc_set_roi = NULL;
    ops->enc_set_srd = NULL;
    ops->enc_set_stream_buf_size = NULL;
    ops->enc_set_super_frame = NULL;
    ops->enc_unregister_channel = NULL;
    ops->fs_chn_stat_query = NULL;
    ops->fs_create_channel = NULL;
    ops->fs_destroy_channel = NULL;
    ops->fs_disable_channel = NULL;
    ops->fs_disable_chn_undistort = NULL;
    ops->fs_enable_channel = NULL;
    ops->fs_enable_chn_undistort = NULL;
    ops->fs_get_delay = NULL;
    ops->fs_get_fifo = NULL;
    ops->fs_get_frame = NULL;
    ops->fs_get_frame_depth = NULL;
    ops->fs_get_max_delay = NULL;
    ops->fs_get_pool = NULL;
    ops->fs_get_timed_frame = NULL;
    ops->fs_release_frame = NULL;
    ops->fs_set_channel_attr = NULL;
    ops->fs_set_delay = NULL;
    ops->fs_set_fifo = NULL;
    ops->fs_set_frame_depth = NULL;
    ops->fs_set_frame_offset = NULL;
    ops->fs_set_max_delay = NULL;
    ops->fs_set_pool = NULL;
    ops->fs_set_rotation = NULL;
    ops->fs_snap_frame = NULL;
    ops->isp_osd_create_region = NULL;
    ops->isp_osd_destroy_region = NULL;
    ops->isp_osd_set_mask = NULL;
    ops->isp_osd_set_pool_size = NULL;
    ops->isp_osd_set_region_attr = NULL;
    ops->isp_osd_show_region = NULL;
    ops->ivs_create_base_move_interface = NULL;
    ops->ivs_create_channel = NULL;
    ops->ivs_create_group = NULL;
    ops->ivs_create_jzdl_interface = NULL;
    ops->ivs_create_move_interface = NULL;
    ops->ivs_create_persondet_interface = NULL;
    ops->ivs_destroy_base_move_interface = NULL;
    ops->ivs_destroy_channel = NULL;
    ops->ivs_destroy_group = NULL;
    ops->ivs_destroy_jzdl_interface = NULL;
    ops->ivs_destroy_move_interface = NULL;
    ops->ivs_destroy_persondet_interface = NULL;
    ops->ivs_get_param = NULL;
    ops->ivs_get_result = NULL;
    ops->ivs_poll_result = NULL;
    ops->ivs_register_channel = NULL;
    ops->ivs_release_data = NULL;
    ops->ivs_release_result = NULL;
    ops->ivs_set_param = NULL;
    ops->ivs_start = NULL;
    ops->ivs_stop = NULL;
    ops->ivs_unregister_channel = NULL;
    ops->osd_attach_to_group = NULL;
    ops->osd_create_group = NULL;
    ops->osd_create_region = NULL;
    ops->osd_destroy_group = NULL;
    ops->osd_destroy_region = NULL;
    ops->osd_get_group_region_attr = NULL;
    ops->osd_get_region_attr = NULL;
    ops->osd_register_region = NULL;
    ops->osd_set_pool_size = NULL;
    ops->osd_set_region_attr = NULL;
    ops->osd_set_region_attr_with_timestamp = NULL;
    ops->osd_show = NULL;
    ops->osd_show_region = NULL;
    ops->osd_start = NULL;
    ops->osd_stop = NULL;
    ops->osd_unregister_region = NULL;
    ops->osd_update_region_data = NULL;
    ops->unbind = NULL;

    ops->enc_create_channel = v4l2_ops_enc_create_channel;
    ops->enc_destroy_channel = v4l2_ops_enc_destroy_channel;
    ops->enc_start = v4l2_ops_enc_start;
    ops->enc_stop = v4l2_ops_enc_stop;
    ops->enc_poll = v4l2_ops_enc_poll;
    ops->enc_get_frame = v4l2_ops_enc_get_frame;
    ops->enc_release_frame = v4l2_ops_enc_release_frame;
    ops->enc_request_idr = v4l2_ops_enc_request_idr;
    ops->deinit = v4l2_ops_deinit;

    built = true;
    return &table;
}
