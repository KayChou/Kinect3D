#pragma once
#include "common.h"

class overlap_removal {
public:

    void init(FIFO<framePacket>** input, FIFO<framePacket>** output);
    void loop(Context *context);

private:
    int left_idx;
    int right_idx;

public:
    FIFO<framePacket>** input;
    FIFO<framePacket>** output;
};


class twoViewRemoval {
public:
    void init();
    void set_io(framePacket* left, framePacket* right);
    void loop();

public:
    std::atomic<bool> processable; //unlocked if uyuv has been prepared
    std::atomic<bool> readable;    //unlocked if yuv has been converted
    std::atomic<bool> is_active;

    framePacket* left; 
    framePacket* right;
};
