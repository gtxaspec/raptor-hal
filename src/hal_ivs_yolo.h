/*
 * hal_ivs_yolo.h -- YOLOv5 head decode and NMS shared by the detection
 * backends (JZDL on the CPU, Uranus on the NNA); adapted from the
 * magik-toolkit sample, as it was in hal_ivs_jzdl.cpp.
 *
 * A head level is the raw output of one detection convolution in NHWC
 * order: for each grid cell, box_num anchors of (x, y, w, h, objectness,
 * class scores...), all before the sigmoid.
 */

#ifndef HAL_IVS_YOLO_H
#define HAL_IVS_YOLO_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

struct DetBox {
    float x1, y1, x2, y2;
    float score;
    int classid;
};

static inline float yolo_sigmoid(float v)
{
    return 1.0f / (1.0f + expf(-v));
}

/* YOLOv5's stock anchors, three w h pairs per level: strides 8, 16, 32. */
static const float yolo_stock_anchors[3][6] = {
    {10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}};

/* A model's anchors per level (strides 8, 16, 32), in model input pixels. */
struct YoloAnchors {
    float wh[3][6];
};

static inline void yolo_anchors_stock(YoloAnchors *a)
{
    std::copy(&yolo_stock_anchors[0][0], &yolo_stock_anchors[0][0] + 18, &a->wh[0][0]);
}

/*
 * Apply the "anchors <stride> w0 h0 w1 h1 w2 h2" lines of text to a, one
 * line per stride (8, 16 or 32), the form of onna's model graphs; other
 * lines are skipped. Returns the number of levels set, or -1 (a left as it
 * was) when an anchors line is malformed or names another stride.
 */
static inline int yolo_parse_anchors(const char *text, YoloAnchors *a)
{
    YoloAnchors t = *a;
    int set = 0;
    for (const char *line = text; line && *line;) {
        const char *end = std::strchr(line, '\n');
        std::string l(line, end ? (size_t)(end - line) : std::strlen(line));
        line = end ? end + 1 : NULL;
        if (l.compare(0, 8, "anchors ") != 0 && l.compare(0, 8, "anchors\t") != 0)
            continue;
        int stride;
        float v[6];
        char extra;
        if (std::sscanf(l.c_str() + 8, "%d %f %f %f %f %f %f %c", &stride, &v[0], &v[1], &v[2],
                        &v[3], &v[4], &v[5], &extra) != 7)
            return -1;
        int level = stride == 8 ? 0 : stride == 16 ? 1 : stride == 32 ? 2 : -1;
        if (level < 0)
            return -1;
        for (int k = 0; k < 6; k++) {
            if (!(v[k] > 0))
                return -1;
            t.wh[level][k] = v[k];
        }
        set++;
    }
    *a = t;
    return set;
}

/*
 * Decode one head level with its three anchors (w h pairs in model input
 * pixels); gw x gh is the level's grid. Boxes are appended in model input
 * pixels.
 *
 * A cell whose objectness is below conf_thresh cannot pass (the class
 * score is at most 1), so its class scores are not read; the best class
 * is picked on the raw scores, the sigmoid being monotonic.
 */
static inline void yolo_decode_anchors(const float *p, const float anchors[6], int gw, int gh,
                                       int input_w, int input_h, float conf_thresh, int num_classes,
                                       std::vector<DetBox> &boxes)
{
    const int box_num = 3;
    const int onechannel = 5 + num_classes;

    for (int h = 0; h < gh; h++) {
        for (int w = 0; w < gw; w++) {
            for (int n = 0; n < box_num; n++) {
                const float *a = p + ((h * gw + w) * box_num + n) * onechannel;

                float obj = yolo_sigmoid(a[4]);
                if (obj < conf_thresh)
                    continue;

                int best_cls = 0;
                for (int c = 1; c < num_classes; c++)
                    if (a[5 + c] > a[5 + best_cls])
                        best_cls = c;
                float prob = obj * yolo_sigmoid(a[5 + best_cls]);
                if (prob < conf_thresh)
                    continue;

                float bx = yolo_sigmoid(a[0]);
                float by = yolo_sigmoid(a[1]);
                float bw = yolo_sigmoid(a[2]);
                float bh = yolo_sigmoid(a[3]);

                float aw = anchors[n * 2] / input_w;
                float ah = anchors[n * 2 + 1] / input_h;

                float box_w = (bw * 2.0f) * (bw * 2.0f) * aw;
                float box_h = (bh * 2.0f) * (bh * 2.0f) * ah;
                float cx = (bx * 2.0f - 0.5f + w) / gw;
                float cy = (by * 2.0f - 0.5f + h) / gh;

                DetBox d;
                d.x1 = (cx - box_w * 0.5f) * input_w;
                d.y1 = (cy - box_h * 0.5f) * input_h;
                d.x2 = (cx + box_w * 0.5f) * input_w;
                d.y2 = (cy + box_h * 0.5f) * input_h;
                d.score = prob;
                d.classid = best_cls;
                boxes.push_back(d);
            }
        }
    }
}

/* Decode one head level with YOLOv5's stock anchors: level 0, 1, 2 are strides 8, 16, 32. */
static inline void yolo_decode_level(const float *p, int level, int gw, int gh, int input_w,
                                     int input_h, float conf_thresh, int num_classes,
                                     std::vector<DetBox> &boxes)
{
    yolo_decode_anchors(p, yolo_stock_anchors[level], gw, gh, input_w, input_h, conf_thresh,
                        num_classes, boxes);
}

/* Class-aware hard NMS, highest score first. */
static inline void yolo_nms(std::vector<DetBox> &input, std::vector<DetBox> &output,
                            float nms_thresh)
{
    std::sort(input.begin(), input.end(),
              [](const DetBox &a, const DetBox &b) { return a.score > b.score; });

    std::vector<bool> merged(input.size(), false);
    for (size_t i = 0; i < input.size(); i++) {
        if (merged[i])
            continue;
        output.push_back(input[i]);
        float area_i = (input[i].x2 - input[i].x1) * (input[i].y2 - input[i].y1);

        for (size_t j = i + 1; j < input.size(); j++) {
            if (merged[j] || input[j].classid != input[i].classid)
                continue;
            float ix0 = std::max(input[i].x1, input[j].x1);
            float iy0 = std::max(input[i].y1, input[j].y1);
            float ix1 = std::min(input[i].x2, input[j].x2);
            float iy1 = std::min(input[i].y2, input[j].y2);
            float iw = std::max(0.0f, ix1 - ix0);
            float ih = std::max(0.0f, iy1 - iy0);
            float inter = iw * ih;
            float area_j = (input[j].x2 - input[j].x1) * (input[j].y2 - input[j].y1);
            if (inter / (area_i + area_j - inter) > nms_thresh)
                merged[j] = true;
        }
    }
}

/*
 * YOLOv5's letterbox: the frame scaled to fit the model input with its
 * aspect kept and centred, the rest padded. Model pixels are frame pixels
 * times scale plus pad.
 */
struct YoloLetterbox {
    float scale, pad_x, pad_y;
};

static inline YoloLetterbox yolo_letterbox(int frame_w, int frame_h, int input_w, int input_h)
{
    YoloLetterbox lb;
    lb.scale = std::min((float)input_w / frame_w, (float)input_h / frame_h);
    lb.pad_x = (input_w - frame_w * lb.scale) / 2;
    lb.pad_y = (input_h - frame_h * lb.scale) / 2;
    return lb;
}

/* A box in model pixels back to frame pixels, clamped to the frame. */
static inline void yolo_unletterbox(const YoloLetterbox &lb, const DetBox &d, int frame_w,
                                    int frame_h, int out[4])
{
    int x0 = (int)((d.x1 - lb.pad_x) / lb.scale);
    int y0 = (int)((d.y1 - lb.pad_y) / lb.scale);
    int x1 = (int)((d.x2 - lb.pad_x) / lb.scale);
    int y1 = (int)((d.y2 - lb.pad_y) / lb.scale);
    out[0] = x0 < 0 ? 0 : x0;
    out[1] = y0 < 0 ? 0 : y0;
    out[2] = x1 >= frame_w ? frame_w - 1 : x1;
    out[3] = y1 >= frame_h ? frame_h - 1 : y1;
}

#endif /* HAL_IVS_YOLO_H */
