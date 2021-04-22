#include <cuda.h>
#include <device_launch_parameters.h>
#include <cuda_runtime_api.h>
#include "FIFO.h"

#define patch_size 2
#define removal_det_region 30
#define removal_det_K removal_det_region * removal_det_region * 3 / 4


extern "C" void Transform(int width_d, int height_d, framePacket *packet, std::vector<std::vector<float>> &R, std::vector<float> &T, TransformStruct *transformStruct);

extern "C" Context_gpu* create_context(Context* ctx_cpu);

extern "C" void updata_context(Context_gpu *ctx_gpu, Context *ctx_cpu);

extern "C" void overlap_removal_cuda(Context_gpu* ctx_gpu, framePacket** frameList, float* dpeth_out);

extern "C" TransformStruct *Transform_gpu_init();

extern "C" void get_matched_points_cuda(Context_gpu* ctx_gpu, Point3fRGB *verts1, Point3fRGB *verts2, float *depth_right, int cam_idx, 
                                        std::vector<Point> &data_r, std::vector<Point> &data_g, std::vector<Point> &data_b);