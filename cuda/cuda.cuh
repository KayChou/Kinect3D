#include <cuda.h>
#include <device_launch_parameters.h>
#include <cuda_runtime_api.h>
#include "FIFO.h"

extern "C" void Transform(int width_d, int height_d, framePacket *packet, std::vector<std::vector<float>> &R, std::vector<float> &T);

extern "C" Context_gpu* create_context(Context* ctx_cpu);

extern "C" void updata_context(Context_gpu *ctx_gpu, Context *ctx_cpu);

extern "C" void overlap_removal_cuda(Context_gpu* ctx_gpu, framePacket** frameList, float* dpeth_out);