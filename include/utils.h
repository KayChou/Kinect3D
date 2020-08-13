#pragma once
#include "common.h"

bool saveRGBDFIFO2Image(FIFO **input, int numOfKinects);

bool saveRGBDFIFO2PLY(FIFO **input, int numOfKinects);

std::vector<float> RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T);