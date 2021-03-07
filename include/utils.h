#pragma once
#include "common.h"

void inv_Matrix_3x3(std::vector<std::vector<float>> R, std::vector<std::vector<float>> &invR);

void saveKRT(Context *context);

bool loadKRT(Context *context);

void savePacket2Bin(framePacket *input, const char* filename);

bool saveRGBDFIFO2Image(FIFO<framePacket> **input, int numOfKinects);

bool saveRGBDFIFO2PLY(FIFO<framePacket> **input, int numOfKinects);

std::vector<float> RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T);

float get_time_diff_ms(timeval start, timeval end);
