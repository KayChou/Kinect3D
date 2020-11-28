#pragma once
#include "common.h"
#include "calibration.h"

extern "C" void Transform(int width_d, int height_d, framePacket *packet, std::vector<std::vector<float>> &R, std::vector<float> &T);

class RGBD_FIFO_Process{
public:

    void process(Context *context);
    void init(FIFO<framePacket>* input, FIFO<frameMesh>* output_pcd, FIFO<framePacket>* output_qt);

    void RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T);

private:
    Calibration calibrate;
    std::vector<float> T;
	std::vector<std::vector<float>> R;

public:
    cv::Mat color;
    cv::Mat registered;
    FIFO<framePacket>* input;
    FIFO<framePacket>* output_qt;
    FIFO<frameMesh>* output_pcd;
    clock_t start=clock(), end=clock();
};
