#pragma once
#include "common.h"
#include "utils.h"

extern "C" Context_gpu* create_context(Context* ctx_cpu);

extern "C" void updata_context(Context_gpu *ctx_gpu, Context *ctx_cpu);

extern "C" void overlap_removal_cuda(Context_gpu* ctx_gpu, framePacket** frameList, float* dpeth_out);


class overlap_removal {
public:

    void init(FIFO<framePacket>** input, FIFO<framePacket>** output, Context *context, Context_gpu *ctx_gpu);
    void loop();

private:
    int left_idx;
    int right_idx;

public:
    Context* context;
    Context_gpu* ctx_gpu;
    framePacket** frameList;
    FIFO<framePacket>** input;
    FIFO<framePacket>** output;

public:
    std::string filename;
    int cnt;
};
