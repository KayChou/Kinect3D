#include "common.h"

#include <string>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>

namespace libfreenect2
{
class Alignment
{
public:
    Alignment(Freenect2Device::IrCameraParams depth_p, Freenect2Device::ColorCameraParams rgb_p);

    void apply(int dx, int dy, float dz, float& cx, float &cy) const;
    void apply(const Frame* rgb, const Frame* depth, Frame* registered, const bool enable_filter, Frame* bigdepth, int* color_depth_map = 0) const;
    void undistortDepth(const Frame *depth, Frame *undistorted) const;
    void getPointXYZRGB (const Frame* undistorted, const Frame* registered, int r, int c, float& x, float& y, float& z, float& rgb) const;
    void getPointXYZ (const Frame* undistorted, int r, int c, float& x, float& y, float& z) const;
    void distort(int mx, int my, float& dx, float& dy) const;
    void depth_to_color(float mx, float my, float& rx, float& ry) const;

    void gen_undistorted_LR(const Frame *depth, Frame *undistorted);
    void bilinearSR(float* depth_LR, float* depth_HR, int tarW, int tarH);
    void get_depth_to_color_mapper(int tarW, int tarH);

private:
    float depth_q;
    float color_q;

    float x_ratio;
    float y_ratio;
    Freenect2Device::IrCameraParams depth;    ///< Depth camera parameters.
    Freenect2Device::ColorCameraParams color; ///< Color camera parameters.

    int distort_map[512 * 424];
    float depth_to_color_map_x[Width_depth_HR * Height_depth_HR];
    float depth_to_color_map_y[Width_depth_HR * Height_depth_HR];
    int depth_to_color_map_yi[Width_depth_HR * Height_depth_HR];

    const int filter_width_half;
    const int filter_height_half;
    const float filter_tolerance;
};
}