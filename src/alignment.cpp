#include "alignment.h"

namespace libfreenect2
{

Alignment::Alignment(Freenect2Device::IrCameraParams depth_p, Freenect2Device::ColorCameraParams rgb_p):
    depth(depth_p), color(rgb_p), filter_width_half(2), filter_height_half(1), filter_tolerance(0.01f)
{
    depth_q = 0.01;
    color_q = 0.002199;
    get_depth_to_color_mapper(Width_depth_HR, Height_depth_HR);
}


//=======================================================================================
// de-distortion for low resolution depth frame
//=======================================================================================
void Alignment::gen_undistorted_LR(const Frame *depth, Frame *undistorted)
{
    float mx, my;
    int ix, iy, index;
    int *map_dist = distort_map;
    float *undistorted_data = (float *)undistorted->data;

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
        }
    }

    const float *depth_data = (float*)depth->data;
    int size_depth = 512 * 424;
    map_dist = distort_map;

    for(int i = 0; i < size_depth; ++i, ++map_dist) {
        const int index = *map_dist; // getting index of distorted depth pixel

        if(index < 0) {  // check if distorted depth pixel is outside of the depth image
            undistorted_data[i] = 0;
            continue;
        }

        // getting depth value for current pixel
        const float z = depth_data[index];
        undistorted_data[i] = z;

        // checking for invalid depth value
        if(z <= 0.0f){
            continue;
        }
    }
}


//=======================================================================================
// depth super-resolution(LR to HR)
//=======================================================================================
void Alignment::bilinearSR(float* depth_LR, float* depth_HR, int tarW, int tarH)
{
    int w = 512;
    int h = 424;
    const float *depth_data = depth_LR;
    float* output_data = depth_HR;

    int x, y, idx;
    float x_diff, y_diff, a, b, c, d, output;

    for(int i = 0; i < tarH; i++) {
        for(int j = 0; j < tarW; j++) {
            x = (int)(x_ratio * j);
            y = (int)(y_ratio * i);

            x_diff = (x_ratio * j) - x;
			y_diff = (y_ratio * i) - y;

            idx = y * w + x;

            a = depth_data[idx];
            b = depth_data[idx + 1];
            c = depth_data[idx + w];
            d = depth_data[idx + w + 1];

            output = a * (1 - x_diff) * (1 - y_diff) + b * x_diff * (1 - y_diff) + c * y_diff * (1 - x_diff) + d * x_diff * y_diff;

            output_data[i * tarW + j] = output;
        }
    }
}


//=======================================================================================
// for each pixel in HR depth frame, compute corresponding color idx
//=======================================================================================
void Alignment::get_depth_to_color_mapper(int tarW, int tarH)
{
    float* map_x = depth_to_color_map_x;
    float* map_y = depth_to_color_map_y;
    int* map_yi = depth_to_color_map_yi;
    float rx, ry;
    
    x_ratio = (float)(512 - 1) / (float)tarW;
    y_ratio = (float)(424 - 1) / (float)tarH;

    for (int y = 0; y < tarH; y++) {
        for (int x = 0; x < tarW; x++) {
            depth_to_color(x * x_ratio, y * y_ratio, rx, ry);
            *map_x++ = rx;
            *map_y++ = ry;
            *map_yi++ = (int)(ry + 0.5f); // compute the y offset to minimize later computations
        }
    }
}


//=======================================================================================
// gen registered image according rgb and de-distortioned depth
//=======================================================================================
void Alignment::apply(const Frame *rgb, const Frame *depth, Frame *registered, const bool enable_filter, Frame *bigdepth, int *color_depth_map) const
{
    // Check if all frames are valid and have the correct size
    if (!rgb || !depth || !registered)
        return;

    float *depth_data = (float*)depth->data;
    const unsigned int *rgb_data = (unsigned int*)rgb->data;

    unsigned int *registered_data = (unsigned int*)registered->data;

    const int *map_dist = distort_map; // 512 * 424
    const float *map_x = depth_to_color_map_x; // Width_depth_HR * Height_depth_HR
    const int *map_yi = depth_to_color_map_yi; // Width_depth_HR * Height_depth_HR

    const int size_depth = Width_depth_HR * Height_depth_HR;
    const int size_color = 1920 * 1080;
    const float color_cx = color.cx + 0.5f; // 0.5f added for later rounding

    // size of filter map with a border of filter_height_half on top and bottom so that no check for borders is needed.
    // since the color image is wide angle no border to the sides is needed.
    const int size_filter_map = size_color + 1920 * filter_height_half * 2; //filter_height_half = 1
    // offset to the important data
    const int offset_filter_map = 1920 * filter_height_half;

    // map for storing the min z values used for each color pixel
    float *filter_map = NULL;
    // pointer to the beginning of the important data
    float *p_filter_map = NULL;

    // map for storing the color offset for each depth pixel
    int *depth_to_c_off = color_depth_map ? color_depth_map : new int[size_depth];
    int *map_c_off = depth_to_c_off;

    // initializing the depth_map with values outside of the Kinect2 range
    if(enable_filter) {
        // printf("init filter map\n");
        filter_map = bigdepth ? (float*)bigdepth->data : new float[size_filter_map];
        p_filter_map = filter_map + offset_filter_map;

        for(float *it = filter_map, *end = filter_map + size_filter_map; it != end; ++it){
            *it = std::numeric_limits<float>::infinity();
        }
        // printf("finish init filter map\n");
    }

    // iterating over all pixels from undistorted depth and registered color image
    // the four maps have the same structure as the images, so their pointers are increased each iteration as well
    for(int i = 0; i < size_depth; ++i, ++map_x, ++map_yi, ++map_c_off) {
        // getting depth value for current pixel
        // printf("iter pixel %d \n", i);
        const float z = depth_data[i];

        // checking for invalid depth value
        if(z <= 0.0f) {
            *map_c_off = -1;
            // printf("z continue\n");
            continue;
        }
        // printf("enter apply: %d\n", i);

        // calculating x offset for rgb image based on depth value
        const float rx = (*map_x + (color.shift_m / z)) * color.fx + color_cx;
        const int cx = rx; // same as round for positive numbers (0.5f was already added to color_cx)
        // getting y offset for depth image
        const int cy = *map_yi;
        // combining offsets
        const int c_off = cx + cy * 1920;

        // check if c_off is outside of rgb image
        // checking rx/cx is not needed because the color image is much wider then the depth image
        if(c_off < 0 || c_off >= size_color) {
            *map_c_off = -1;
            // printf("continue\n");
            continue;
        }

        // saving the offset for later
        *map_c_off = c_off;
        // printf("c_off: %d\n", c_off);

        if(enable_filter) {
            // printf("begin filter\n");
            // setting a window around the filter map pixel corresponding to the color pixel with the current z value
            int yi = (cy - filter_height_half) * 1920 + cx - filter_width_half; // index of first pixel to set
            for(int r = -filter_height_half; r <= filter_height_half; ++r, yi += 1920) {// index increased by a full row each iteration
                float *it = p_filter_map + yi;
                for(int c = -filter_width_half; c <= filter_width_half; ++c, ++it) {
                    // only set if the current z is smaller
                    if(z < *it) {
                        *it = z;
                    }
                }
            }
            // printf("finish filter\n");
        }
    }

    /* Construct 'registered' image. */

    // reseting the pointers to the beginning
    map_c_off = depth_to_c_off;

    // printf("enter apply: filter\n");
    /* Filter drops duplicate pixels due to aspect of two cameras. */
    if(enable_filter) {
        // run through all registered color pixels and set them based on filter results
        for(int i = 0; i < size_depth; ++i, ++map_c_off, ++registered_data) {
            const int c_off = *map_c_off;

            // check if offset is out of image
            if(c_off < 0) {
                *registered_data = 0;
                continue;
            }

            const float min_z = p_filter_map[c_off];
            const float z = depth_data[i];

            // check for allowed depth noise
            *registered_data = (z - min_z) / z > filter_tolerance ? 0 : *(rgb_data + c_off);
        }

        if (!bigdepth) delete[] filter_map;
    }
    else {
        // run through all registered color pixels and set them based on c_off
        for(int i = 0; i < size_depth; ++i, ++map_c_off, ++registered_data) {
            const int c_off = *map_c_off;

            // check if offset is out of image
            *registered_data = c_off < 0 ? 0 : *(rgb_data + c_off);
        }
    }
    if (!color_depth_map) delete[] depth_to_c_off;
}


void Alignment::apply(int dx, int dy, float dz, float& cx, float &cy) const
{
    const int index = dx + dy * 512;
    float rx = depth_to_color_map_x[index];
    cy = depth_to_color_map_y[index];

    rx += (color.shift_m / dz);
    cx = rx * color.fx + color.cx;
}


void Alignment::getPointXYZRGB(const Frame* undistorted, const Frame* registered, int r, int c, float& x, float& y, float& z, float& rgb) const
{
    getPointXYZ(undistorted, r, c, x, y, z);

    if(isnan(z)) {
        rgb = 0;
    }
    else {
        float* registered_data = (float *)registered->data;
        rgb = registered_data[Width_depth_HR * r + c];
    }
}


void Alignment::getPointXYZ(const Frame *undistorted, int r, int c, float &x, float &y, float &z) const
{
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const float cx(depth.cx), cy(depth.cy);
    const float fx(1/depth.fx), fy(1/depth.fy);
    float* undistorted_data = (float *)undistorted->data;
    const float depth_val = undistorted_data[Width_depth_HR * r + c]/1000.0f; //scaling factor, so that value of 1 is one meter.
    if (isnan(depth_val) || depth_val <= 0.001) {
        //depth value is not valid
        x = y = z = bad_point;
    }
    else {
        x = (c * x_ratio + 0.5 - cx) * fx * depth_val;
        y = (r * y_ratio + 0.5 - cy) * fy * depth_val;
        z = depth_val;
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