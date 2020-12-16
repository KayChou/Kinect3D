#pragma once
#include "common.h"
#include "calibration.h"
#include "utils.h"

extern "C" void Transform(int width_d, int height_d, framePacket *packet, std::vector<std::vector<float>> &R, std::vector<float> &T);

class Transform2world{
public:

    void process(Context *context);
    void init(int idx, FIFO<framePacket>* input, FIFO<framePacket>* output, FIFO<framePacket>* output_qt);

    void RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T);

private:
    int idx;
    Calibration calibrate;
    std::vector<float> T;
	std::vector<std::vector<float>> R;

public:
    cv::Mat color;
    cv::Mat registered;
    FIFO<framePacket>* input;
    FIFO<framePacket>* output_qt;
    FIFO<framePacket>* output;
    clock_t start = clock(), end = clock();
};
