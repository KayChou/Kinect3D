#pragma once

#include <stdio.h>
#include <stdint.h>
#include "config.h"
#include "typedef.h"

class registeration
{
public:
    ColorCameraParams param_color;
    IrCameraParams param_ir;

public:
    float* depth_undistort;

public:
    registeration();
    ~registeration();
    void init(ColorCameraParams param_color, IrCameraParams param_ir);
    void register_rgbd(uint8_t *color, float* depth, uint8_t* rgbd);
    void undistort_depth(float* depth, float* depth_undistort);

private:
    void distort(int mx, int my, float &x, float &y);
    void depth_to_color(float mx, float my, float &rx, float &ry, float depth_val);
};