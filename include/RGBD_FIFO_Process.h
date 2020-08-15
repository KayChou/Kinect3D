#pragma once
#include "common.h"
#include "calibration.h"

void startAll_RGBD_FIFO_Process(FIFO<framePacket>** input, FIFO<framePacket> **output_pd, FIFO<framePacket> **output_qt, bool &calibrationFlag);
void destroyAll_RGBD_FIFO_Process();


class RGBD_FIFO_Process{
public:
    RGBD_FIFO_Process(FIFO<framePacket>* input, FIFO<framePacket>* output_qt);
    ~RGBD_FIFO_Process();

    void process(bool* calibrationFlag);
    void destory();

    bool getFinishFlag();

private:
    Calibration calibrate;

public:
    cv::Mat color;
    cv::Mat registered;
    FIFO<framePacket>* input_;
    FIFO<framePacket>* output_qt;
    FIFO<framePacket>* output_pd;
    clock_t start=clock(), end=clock();

    bool finishFlag;
};
