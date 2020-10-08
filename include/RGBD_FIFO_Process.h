#pragma once
#include "common.h"
#include "calibration.h"

class RGBD_FIFO_Process{
public:

    void process(Context *context);
    void init(FIFO<framePacket>* input, FIFO<framePacket>* output_pcd, FIFO<framePacket>* output_qt);

private:
    Calibration calibrate;

public:
    cv::Mat color;
    cv::Mat registered;
    FIFO<framePacket>* input;
    FIFO<framePacket>* output_qt;
    FIFO<framePacket>* output_pcd;
    clock_t start=clock(), end=clock();
};
