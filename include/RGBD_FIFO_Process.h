#pragma once
#include "common.h"

void startAll_RGBD_FIFO_Process(FIFO<framePacket>** input, FIFO<framePacket> *output, bool &calibrationFlag);
void destroyAll_RGBD_FIFO_Process();


class RGBD_FIFO_Process{
public:
    RGBD_FIFO_Process(FIFO<framePacket>* input, FIFO<framePacket>* output=NULL);
    ~RGBD_FIFO_Process();

    void process(bool* calibrationFlag);
    void destory();

public:
    cv::Mat color;
    cv::Mat registered;
    FIFO<framePacket>* input_;
    FIFO<framePacket>* output_;
    clock_t start=clock(), end=clock();
};
