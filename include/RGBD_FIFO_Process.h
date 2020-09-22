#pragma once
#include "common.h"
#include "calibration.h"

void startAll_RGBD_FIFO_Process(FIFO<framePacket>** input, FIFO<framePacket> **output_pcd, FIFO<framePacket> **output_qt, Context *context);
void destroyAll_RGBD_FIFO_Process();


class RGBD_FIFO_Process{
public:
    RGBD_FIFO_Process(FIFO<framePacket>* input, FIFO<framePacket>* output_pcd, FIFO<framePacket>* output_qt);
    ~RGBD_FIFO_Process();

    void process(Context *context);
    void destory();

    bool getFinishFlag();

private:
    Calibration calibrate;

public:
    cv::Mat color;
    cv::Mat registered;
    FIFO<framePacket>* input_;
    FIFO<framePacket>* output_qt;
    FIFO<framePacket>* output_pcd;
    clock_t start=clock(), end=clock();

    bool finishFlag;
};
