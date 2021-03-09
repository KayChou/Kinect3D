#include "cuda.cuh"

#define patch_size 8

Context_gpu* create_context(Context* ctx_cpu) {
    cudaSetDevice(0);

    Context_gpu *ctx_gpu;
    cudaMallocManaged((void**)&ctx_gpu, sizeof(Context_gpu));

    for(int i=0; i<numKinects; i++) {
        cudaMallocManaged((void**)&ctx_gpu->R[i], sizeof(float) * 9);
        cudaMallocManaged((void**)&ctx_gpu->T[i], sizeof(float) * 3);
        cudaMallocManaged((void**)&ctx_gpu->invR[i], sizeof(float) * 9);
        cudaMalloc((void**)&ctx_gpu->vertices[i], sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR);
        cudaMalloc((void**)&ctx_gpu->depth[i], sizeof(float) * Width_depth_HR * Height_depth_HR);
        cudaMalloc((void**)&ctx_gpu->vertices_shift[i], sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR);
        cudaMalloc((void**)&ctx_gpu->depth_shift[i], sizeof(float) * Width_depth_HR * Height_depth_HR);
    }
    ctx_gpu->width = Width_depth_HR;
    ctx_gpu->height = Height_depth_HR;
    ctx_gpu->x_ratio = Width_depth_HR / 512.f;
    ctx_gpu->y_ratio = Height_depth_HR / 424.f;
    
    return ctx_gpu;
}


void updata_context(Context_gpu *ctx_gpu, Context *ctx_cpu) {
    for(int i=0; i<numKinects; i++) {
        cudaMemcpy(&ctx_gpu->R[i][0], &ctx_cpu->R[i][0][0], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][1], &ctx_cpu->R[i][0][1], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][2], &ctx_cpu->R[i][0][2], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][3], &ctx_cpu->R[i][1][0], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][4], &ctx_cpu->R[i][1][1], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][5], &ctx_cpu->R[i][1][2], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][6], &ctx_cpu->R[i][2][0], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][7], &ctx_cpu->R[i][2][1], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->R[i][8], &ctx_cpu->R[i][2][2], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->T[i][0], &ctx_cpu->T[i][0], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->T[i][1], &ctx_cpu->T[i][1], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->T[i][2], &ctx_cpu->T[i][2], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][0], &ctx_cpu->invR[i][0][0], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][1], &ctx_cpu->invR[i][0][1], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][2], &ctx_cpu->invR[i][0][2], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][3], &ctx_cpu->invR[i][1][0], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][4], &ctx_cpu->invR[i][1][1], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][5], &ctx_cpu->invR[i][1][2], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][6], &ctx_cpu->invR[i][2][0], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][7], &ctx_cpu->invR[i][2][1], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->invR[i][8], &ctx_cpu->invR[i][2][2], sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->K[i].fx, &ctx_cpu->K[i].fx, sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->K[i].fy, &ctx_cpu->K[i].fy, sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->K[i].cx, &ctx_cpu->K[i].cx, sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(&ctx_gpu->K[i].cy, &ctx_cpu->K[i].cy, sizeof(float), cudaMemcpyHostToDevice);
    }
}


__device__ void backforward_mapping(float *point, float *R, float *T)
{
    float res[3];
    res[0] = point[0] * R[0] + point[1] * R[1] + point[2] * R[2];
    res[1] = point[0] * R[3] + point[1] * R[4] + point[2] * R[5];
    res[2] = point[0] * R[6] + point[1] * R[7] + point[2] * R[8];

    point[0] = res[0] - T[0];
    point[1] = res[1] - T[1];
    point[2] = res[2] - T[2];
}


__global__ void overlap_removal_kernel(Context_gpu* ctx_gpu, Point3fRGB* left, Point3fRGB* right, float* depth, int idx) {
    int x_idx = threadIdx.x + blockIdx.x * blockDim.x;
    int y_idx = threadIdx.y + blockIdx.y * blockDim.y;
    int pix_idx = x_idx + y_idx * blockDim.x * gridDim.x;

    float tempPoint[3];
    tempPoint[0] = left[pix_idx].X;
    tempPoint[1] = left[pix_idx].Y;
    tempPoint[2] = left[pix_idx].Z;

    backforward_mapping(tempPoint, ctx_gpu->invR[idx], ctx_gpu->T[idx]);

    if(tempPoint[2] <= 0) {
        return;
    }

    int x = (int)(tempPoint[0] * ctx_gpu->x_ratio / tempPoint[2] * ctx_gpu->K[idx].fx + ctx_gpu->K[idx].cx - 0.5);
    int y = (int)(tempPoint[1] * ctx_gpu->y_ratio / tempPoint[2] * ctx_gpu->K[idx].fy + ctx_gpu->K[idx].cy - 0.5);

    bool flag = false;
    int index;

    if(x >= 0 && x < Width_depth_HR && y >= 0 && y < Height_depth_HR) {
        for(int i=0; i<patch_size; i++) {
            for(int j=0; j<patch_size; j++) {
                index = ((y - patch_size/2) + i) * Width_depth_HR + (x - patch_size/2) + j;
                if(index >= 0 && index < Width_depth_HR * Height_depth_HR && tempPoint[2] > (depth[index] + 0.1)) {
                    flag = true;
                }
            }
        }
    }

    if(flag) {
        // if(idx == 0) {
        //     right[pix_idx].R = 255;
        //     right[pix_idx].G = 0;
        //     right[pix_idx].B = 0;
        // }
        // if(idx == 1) {
        //     right[pix_idx].R = 0;
        //     right[pix_idx].G = 255;
        //     right[pix_idx].B = 0;
        // }
        // right[pix_idx].X = 0;
        // right[pix_idx].Y = 0;
        // right[pix_idx].Z = 0;
        right[pix_idx].R = 255;
        right[pix_idx].G = 0;
        right[pix_idx].B = 0;
    }
}


void overlap_removal_cuda(Context_gpu* ctx_gpu, framePacket** frameList) {
#if 1
    cudaEvent_t start, end;
    cudaEventCreate(&start);
    cudaEventCreate(&end);

    cudaEventRecord(start);
#endif
    for(int i=0; i<numKinects; i++) {
        cudaMemcpy(ctx_gpu->vertices[i], frameList[i]->vertices, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
        cudaMemcpy(ctx_gpu->depth[i], frameList[i]->data_d, sizeof(float) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
        cudaMemcpy(ctx_gpu->vertices_shift[(i+1) % numKinects], ctx_gpu->vertices[i], sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyDeviceToDevice);
        cudaMemcpy(ctx_gpu->depth_shift[(i+1) % numKinects], ctx_gpu->depth[i], sizeof(float) * Width_depth_HR * Height_depth_HR, cudaMemcpyDeviceToDevice);
    }

    dim3 blocks(Width_depth_HR / 32, Height_depth_HR / 8);
    dim3 threads(32, 8);

    for(int i=0; i<numKinects; i++) {
        overlap_removal_kernel<<<blocks, threads>>>(ctx_gpu, ctx_gpu->vertices[i], ctx_gpu->vertices_shift[i], ctx_gpu->depth_shift[i], (i+1) % numKinects);
    }

    for(int i=0; i<numKinects; i++) {
        cudaMemcpy(frameList[i]->vertices, ctx_gpu->vertices_shift[(i + numKinects - 1) % numKinects], sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyDeviceToHost);
        // cudaMemcpy(frameList[i]->vertices, ctx_gpu->vertices[i], sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyDeviceToHost);
    }

#if 1
    cudaEventRecord(end);
    cudaEventSynchronize(end);
    float millisecond = 0;
    cudaEventElapsedTime(&millisecond, start, end);
    printf("\t overlap_removal_cuda time = %f ms\n", millisecond);
#endif
}