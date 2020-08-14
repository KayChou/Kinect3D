#pragma once
#include "common.h"

bool saveRGBDFIFO2Image(FIFO<framePacket> **input, int numOfKinects);

bool saveRGBDFIFO2PLY(FIFO<framePacket> **input, int numOfKinects);

std::vector<float> RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T);