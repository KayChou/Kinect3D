#pragma once
#include "common.h"

void startAll_RGBD_FIFO_Process(FIFO** input, QImage* image, bool &calibrationFlag);
void destroyAll_RGBD_FIFO_Process();


class RGBD_FIFO_Process{
public:
    RGBD_FIFO_Process(FIFO* input);
    ~RGBD_FIFO_Process();

    void process(bool* calibrationFlag, QImage* imageOut);
    void destory();

public:
    cv::Mat color;
    cv::Mat registered;
    FIFO* input_;
    clock_t start=clock(), end=clock();
};
