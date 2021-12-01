#include "registeration.h"

registeration::registeration() {
    this->depth_undistort = new float[512 * 424];
}


registeration::~registeration() {
    delete [] this->depth_undistort;
}


void registeration::init(ColorCameraParams param_color, IrCameraParams param_ir) {
    this->param_color = param_color;
    this->param_ir = param_ir;
}


void registeration::register_rgbd(uint8_t *color, float* depth, uint8_t* rgbd) {
    undistort_depth(depth, this->depth_undistort);
    // loop all undistorted depth pixels, get its color
    float mx, my;
    int ix, iy;
    for(int y = 0; y < 424; y++) {
        for(int x = 0; x < 512; x++) {
            int idx = x + y * 512;

            if(this->depth_undistort[idx] <= 0) {
                continue;
            }

            depth_to_color(x, y, mx, my, this->depth_undistort[idx]);
            ix = (int)(mx + 0.5);
            iy = (int)(my + 0.5);

            if(ix < 0 || ix >= 1920 || iy < 0 || iy >= 1080) {
                rgbd[3 * idx + 0] = 0;
                rgbd[3 * idx + 1] = 0;
                rgbd[3 * idx + 2] = 0;
            }
            else {
                rgbd[3 * idx + 0] = color[3 * (ix + 1920 * iy) + 0];
                rgbd[3 * idx + 1] = color[3 * (ix + 1920 * iy) + 1];
                rgbd[3 * idx + 2] = color[3 * (ix + 1920 * iy) + 2];
            }
        }
    }
}


void registeration::undistort_depth(float* depth, float* depth_undistort) {
    float mx, my;
    int ix, iy;
    for(int y = 0; y < 424; y++) {
        for(int x = 0; x < 512; x++) {
            distort(x, y, mx, my); // compute undistorted depth index
            ix = (int)(mx + 0.5f);
            iy = (int)(my + 0.5f);

            int idx = ix + iy * 512;
            if(ix < 0 || ix >= 512 || iy < 0 || iy >= 424) {
                depth_undistort[idx] = -1;
            }
            else {
                depth_undistort[idx] = depth[x + y * 512];
            }
        }
    }
}


void registeration::depth_to_color(float mx, float my, float &rx, float &ry, float depth_val) {
    const float depth_q = 0.01;
    const float color_q = 0.002199;
    mx = (mx - param_ir.cx) * depth_q;
    my = (my - param_ir.cy) * depth_q;

    float wx =
        (mx * mx * mx * param_color.mx_x3y0) + (my * my * my * param_color.mx_x0y3) +
        (mx * mx * my * param_color.mx_x2y1) + (my * my * mx * param_color.mx_x1y2) +
        (mx * mx * param_color.mx_x2y0) + (my * my * param_color.mx_x0y2) + (mx * my * param_color.mx_x1y1) +
        (mx * param_color.mx_x1y0) + (my * param_color.mx_x0y1) + (param_color.mx_x0y0);

    float wy =
        (mx * mx * mx * param_color.my_x3y0) + (my * my * my * param_color.my_x0y3) +
        (mx * mx * my * param_color.my_x2y1) + (my * my * mx * param_color.my_x1y2) +
        (mx * mx * param_color.my_x2y0) + (my * my * param_color.my_x0y2) + (mx * my * param_color.my_x1y1) +
        (mx * param_color.my_x1y0) + (my * param_color.my_x0y1) + (param_color.my_x0y0);

    rx = (wx / (param_color.fx * color_q)) - (param_color.shift_m / param_color.shift_d);
    rx = (rx + param_color.shift_m / depth_val) * param_color.fx + param_color.cx;
    ry = (wy / color_q) + param_color.cy;
}


void registeration::distort(int mx, int my, float &x, float &y) {
    // see http://en.wikipedia.org/wiki/Distortion_(optics) for description
    float dx = ((float)mx - param_ir.cx) / param_ir.fx;
    float dy = ((float)my - param_ir.cy) / param_ir.fy;
    float dx2 = dx * dx;
    float dy2 = dy * dy;
    float r2 = dx2 + dy2;
    float dxdy2 = 2 * dx * dy;
    float kr = 1 + ((param_ir.k3 * r2 + param_ir.k2) * r2 + param_ir.k1) * r2;
    x = param_ir.fx * (dx * kr + param_ir.p2 * (r2 + 2 * dx2) + param_ir.p1 * dxdy2) + param_ir.cx;
    y = param_ir.fy * (dy * kr + param_ir.p1 * (r2 + 2 * dy2) + param_ir.p2 * dxdy2) + param_ir.cy;
}