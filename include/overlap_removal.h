#pragma once
#include "common.h"


class twoViewRemoval {
public:
    void init(Context *context);
    void set_io(framePacket* left, framePacket* right, int idx);
    void loop();
    void backforward_mapping(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T);
    void map_to_pixel(Point3f &point, float fx, float fy, float cx, float cy);

public:
    std::atomic<bool> processable; //unlocked if uyuv has been prepared
    std::atomic<bool> readable;    //unlocked if yuv has been converted
    std::atomic<bool> is_active;

    framePacket* left; // left view's framePacket
    framePacket* right; // right view's framePacket
    int idx;
    Context* context;
};


class overlap_removal {
public:

    void init(FIFO<framePacket>** input, FIFO<framePacket>** output, Context *context);
    void loop();

private:
    int left_idx;
    int right_idx;

public:
    Context* context;
    framePacket** frameList;
    FIFO<framePacket>** input;
    FIFO<framePacket>** output;

public:
    twoViewRemoval removal[numKinects];
    std::thread Thread[numKinects];
};
