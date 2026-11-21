/*
 * test_yolo.cpp -- host tests of the YOLOv5 decode and NMS in
 * src/hal_ivs_yolo.h (shared by the JZDL and Uranus backends).
 *
 *   make -C tests
 */

#include "hal_ivs_yolo.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

static int failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

static bool near(float a, float b)
{
    return fabsf(a - b) < 1e-3f;
}

/* A head level of gw x gh cells, every value raw -10 (sigmoid about 0). */
struct head {
    int gw, gh, nc;
    std::vector<float> v;
    head(int w, int h, int classes) : gw(w), gh(h), nc(classes), v(w * h * 3 * (5 + classes), -10)
    {
    }
    float *anchor(int x, int y, int n)
    {
        return &v[((y * gw + x) * 3 + n) * (5 + nc)];
    }
};

static float sig(float v)
{
    return 1.0f / (1.0f + expf(-v));
}

static void test_empty(void)
{
    head h(8, 6, 4);
    std::vector<DetBox> boxes;
    yolo_decode_level(h.v.data(), 0, h.gw, h.gh, 64, 48, 0.25f, h.nc, boxes);
    CHECK(boxes.empty());
}

/* Raw 0 for x, y, w, h: the box sits on the cell centre at the anchor's size. */
static void test_one_box(void)
{
    head h(8, 6, 4);
    float *a = h.anchor(3, 2, 1);
    a[0] = a[1] = a[2] = a[3] = 0;
    a[4] = 4;     /* objectness */
    a[5 + 2] = 3; /* class 2 */
    a[5 + 1] = 2;
    std::vector<DetBox> boxes;
    yolo_decode_level(h.v.data(), 0, h.gw, h.gh, 64, 48, 0.25f, h.nc, boxes);
    CHECK(boxes.size() == 1);
    if (boxes.size() != 1)
        return;
    const DetBox &d = boxes[0];
    /* stride 8, anchor 1 of level 0 is 16x30; centre (3.5, 2.5) cells */
    CHECK(near(d.x1, 28 - 8) && near(d.x2, 28 + 8));
    CHECK(near(d.y1, 20 - 15) && near(d.y2, 20 + 15));
    CHECK(d.classid == 2);
    CHECK(near(d.score, sig(4) * sig(3)));
}

/* Width and height are the anchor's times (2 sigmoid(raw))^2. */
static void test_box_size(void)
{
    head h(8, 6, 1);
    float *a = h.anchor(0, 0, 0);
    a[0] = a[1] = 0;
    a[2] = 1;
    a[3] = -1;
    a[4] = a[5] = 6;
    std::vector<DetBox> boxes;
    yolo_decode_level(h.v.data(), 0, h.gw, h.gh, 64, 48, 0.5f, h.nc, boxes);
    CHECK(boxes.size() == 1);
    if (boxes.size() == 1) {
        float sw = 2 * sig(1), sh = 2 * sig(-1);
        CHECK(near(boxes[0].x2 - boxes[0].x1, 10 * sw * sw)); /* anchor 0 of level 0: 10x13 */
        CHECK(near(boxes[0].y2 - boxes[0].y1, 13 * sh * sh));
    }
}

/* Level 2 is stride 32 with anchors 116x90, 156x198, 373x326. */
static void test_level_anchors(void)
{
    head h(4, 3, 1);
    float *a = h.anchor(0, 0, 2);
    a[0] = a[1] = a[2] = a[3] = 0;
    a[4] = a[5] = 6;
    std::vector<DetBox> boxes;
    yolo_decode_level(h.v.data(), 2, h.gw, h.gh, 128, 96, 0.5f, h.nc, boxes);
    CHECK(boxes.size() == 1);
    if (boxes.size() == 1) {
        CHECK(near(boxes[0].x2 - boxes[0].x1, 373));
        CHECK(near(boxes[0].y2 - boxes[0].y1, 326));
        CHECK(near(boxes[0].x1 + boxes[0].x2, 32)); /* centre 16 px */
    }
}

/* Objectness above, objectness x class below the threshold: no box. */
static void test_threshold_product(void)
{
    head h(2, 2, 2);
    float *a = h.anchor(1, 1, 0);
    a[4] = 1;  /* 0.73 */
    a[5] = -1; /* 0.27: product 0.20 */
    std::vector<DetBox> boxes;
    yolo_decode_level(h.v.data(), 0, h.gw, h.gh, 16, 16, 0.25f, h.nc, boxes);
    CHECK(boxes.empty());
    a[5] = 1; /* product 0.53 */
    yolo_decode_level(h.v.data(), 0, h.gw, h.gh, 16, 16, 0.25f, h.nc, boxes);
    CHECK(boxes.size() == 1);
}

static DetBox box(float x1, float y1, float x2, float y2, float score, int cls)
{
    DetBox d;
    d.x1 = x1;
    d.y1 = y1;
    d.x2 = x2;
    d.y2 = y2;
    d.score = score;
    d.classid = cls;
    return d;
}

static void test_nms(void)
{
    std::vector<DetBox> in, out;
    in.push_back(box(0, 0, 10, 10, 0.6f, 0));
    in.push_back(box(1, 1, 11, 11, 0.9f, 0));   /* overlaps the first, IoU 0.68 */
    in.push_back(box(1, 1, 11, 11, 0.5f, 1));   /* same place, other class */
    in.push_back(box(20, 20, 30, 30, 0.4f, 0)); /* apart */
    yolo_nms(in, out, 0.45f);
    CHECK(out.size() == 3);
    if (out.size() == 3) {
        CHECK(near(out[0].score, 0.9f) && out[0].classid == 0);
        CHECK(near(out[1].score, 0.5f) && out[1].classid == 1);
        CHECK(near(out[2].score, 0.4f));
    }
    out.clear();
    yolo_nms(in, out, 0.7f); /* IoU 0.68 is under: both class-0 boxes stay */
    CHECK(out.size() == 4);
}

/* The sub stream into a 640x384 model: no scaling, 12 rows of pad each side. */
static void test_letterbox_sub_stream(void)
{
    YoloLetterbox lb = yolo_letterbox(640, 360, 640, 384);
    CHECK(near(lb.scale, 1) && near(lb.pad_x, 0) && near(lb.pad_y, 12));
    int b[4];
    yolo_unletterbox(lb, box(100, 112, 300, 212, 1, 0), 640, 360, b);
    CHECK(b[0] == 100 && b[1] == 100 && b[2] == 300 && b[3] == 200);
}

/* A portrait frame into a square model: scaled down, padded left and right; out-of-frame
 * corners are clamped. */
static void test_letterbox_portrait(void)
{
    YoloLetterbox lb = yolo_letterbox(810, 1080, 640, 640);
    CHECK(near(lb.scale, 640.0f / 1080) && near(lb.pad_x, (640 - 810 * (640.0f / 1080)) / 2) &&
          near(lb.pad_y, 0));
    int b[4];
    /* frame (405, 540) +- 100 px, into model pixels */
    float s = lb.scale;
    yolo_unletterbox(lb, box(305 * s + lb.pad_x, 440 * s, 505 * s + lb.pad_x, 640 * s, 1, 0), 810,
                     1080, b);
    CHECK(abs(b[0] - 305) <= 1 && abs(b[1] - 440) <= 1 && abs(b[2] - 505) <= 1 &&
          abs(b[3] - 640) <= 1);
    yolo_unletterbox(lb, box(0, -5, 640, 645, 1, 0), 810, 1080, b);
    CHECK(b[0] == 0 && b[1] == 0 && b[2] == 809 && b[3] == 1079);
}

int main(void)
{
    test_empty();
    test_one_box();
    test_box_size();
    test_level_anchors();
    test_threshold_product();
    test_nms();
    test_letterbox_sub_stream();
    test_letterbox_portrait();
    printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
