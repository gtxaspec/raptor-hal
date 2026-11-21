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

/*
 * Decode one head level. level 0, 1, 2 are strides 8, 16, 32 and take the
 * stock YOLOv5 anchors of that stride; gw x gh is the level's grid. Boxes
 * are appended in model input pixels.
 *
 * A cell whose objectness is below conf_thresh cannot pass (the class
 * score is at most 1), so its class scores are not read; the best class
 * is picked on the raw scores, the sigmoid being monotonic.
 */
static inline void yolo_decode_level(const float *p, int level, int gw, int gh, int input_w,
                                     int input_h, float conf_thresh, int num_classes,
                                     std::vector<DetBox> &boxes)
{
    static const float anchors[] = {10, 13, 16,  30,  33, 23,  30,  61,  62,
                                    45, 59, 119, 116, 90, 156, 198, 373, 326};
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

                float aw = anchors[level * box_num * 2 + n * 2] / input_w;
                float ah = anchors[level * box_num * 2 + n * 2 + 1] / input_h;

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

#endif /* HAL_IVS_YOLO_H */
