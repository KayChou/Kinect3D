#include "alignment.h"

namespace libfreenect2
{

Alignment::Alignment(Freenect2Device::IrCameraParams depth_p, Freenect2Device::ColorCameraParams rgb_p):
    depth(depth_p), color(rgb_p), filter_width_half(2), filter_height_half(1), filter_tolerance(0.01f)
{
    depth_q = 0.01;
    color_q = 0.002199;

    float mx, my;
    int ix, iy, index;
    float rx, ry;
    int *map_dist = distort_map;
    float *map_x = depth_to_color_map_x;
    float *map_y = depth_to_color_map_y;
    int *map_yi = depth_to_color_map_yi;

    for (int y = 0; y < 424; y++) {
        for (int x = 0; x < 512; x++) {
            distort(x, y, mx, my); // compute the dirstored coordinate for current pixel
            ix = (int)(mx + 0.5f); // rounding the values and check if the pixel is inside the image
            iy = (int)(my + 0.5f);
            if(ix < 0 || ix >= 512 || iy < 0 || iy >= 424) {
                index = -1;
            }
            else {
                index = iy * 512 + ix; // computing the index from the coordianted for faster access to the data
            }
            *map_dist++ = index;

            // compute the depth to color mapping entries for the current pixel
            depth_to_color(x, y, rx, ry);
            *map_x++ = rx;
            *map_y++ = ry;
            *map_yi++ = (int)(ry + 0.5f); // compute the y offset to minimize later computations
        }
    }
}



void Alignment::distort(int mx, int my, float& x, float& y) const
{
    // see http://en.wikipedia.org/wiki/Distortion_(optics) for description
    float dx = ((float)mx - depth.cx) / depth.fx;
    float dy = ((float)my - depth.cy) / depth.fy;
    float dx2 = dx * dx;
    float dy2 = dy * dy;
    float r2 = dx2 + dy2;
    float dxdy2 = 2 * dx * dy;
    float kr = 1 + ((depth.k3 * r2 + depth.k2) * r2 + depth.k1) * r2;
    x = depth.fx * (dx * kr + depth.p2 * (r2 + 2 * dx2) + depth.p1 * dxdy2) + depth.cx;
    y = depth.fy * (dy * kr + depth.p1 * (r2 + 2 * dy2) + depth.p2 * dxdy2) + depth.cy;
}


void Alignment::depth_to_color(float mx, float my, float& rx, float& ry) const
{
    mx = (mx - depth.cx) * depth_q;
    my = (my - depth.cy) * depth_q;

    float wx =
        (mx * mx * mx * color.mx_x3y0) + (my * my * my * color.mx_x0y3) +
        (mx * mx * my * color.mx_x2y1) + (my * my * mx * color.mx_x1y2) +
        (mx * mx * color.mx_x2y0) + (my * my * color.mx_x0y2) + (mx * my * color.mx_x1y1) +
        (mx * color.mx_x1y0) + (my * color.mx_x0y1) + (color.mx_x0y0);

    float wy =
        (mx * mx * mx * color.my_x3y0) + (my * my * my * color.my_x0y3) +
        (mx * mx * my * color.my_x2y1) + (my * my * mx * color.my_x1y2) +
        (mx * mx * color.my_x2y0) + (my * my * color.my_x0y2) + (mx * my * color.my_x1y1) +
        (mx * color.my_x1y0) + (my * color.my_x0y1) + (color.my_x0y0);

    rx = (wx / (color.fx * color_q)) - (color.shift_m / color.shift_d);
    ry = (wy / color_q) + color.cy;
}




}