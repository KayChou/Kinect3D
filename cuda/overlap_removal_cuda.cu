#include "cuda.cuh"

Context_gpu* create_context(Context* ctx_cpu) {
    cudaSetDevice(0);

    Context_gpu *ctx_gpu;
    cudaMallocManaged((void**)&ctx_gpu, sizeof(Context_gpu));
    cudaMallocManaged((void**)&ctx_gpu->depth_out, 512 * 424 * sizeof(float));
    cudaMalloc((void**)&ctx_gpu->mask, sizeof(float) * Width_depth_HR * Height_depth_HR);
    cudaMallocManaged((void**)&ctx_gpu->matched_idx, sizeof(int) * Width_depth_HR * Height_depth_HR);
    cudaMallocManaged((void**)&ctx_gpu->verts1_match, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR);
    cudaMallocManaged((void**)&ctx_gpu->verts2_match, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR);
    cudaMallocManaged((void**)&ctx_gpu->depth_match, sizeof(float) * Width_depth_HR * Height_depth_HR);

    for(int i=0; i<numKinects; i++) {
        cudaMallocManaged((void**)&ctx_gpu->R[i], sizeof(float) * 9);
        cudaMallocManaged((void**)&ctx_gpu->T[i], sizeof(float) * 3);
        cudaMallocManaged((void**)&ctx_gpu->invR[i], sizeof(float) * 9);
        cudaMallocManaged((void**)&ctx_gpu->vertices[i], sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR);
        cudaMalloc((void**)&ctx_gpu->depth[i], sizeof(float) * Width_depth_HR * Height_depth_HR);

        ctx_gpu->color_params[i].Ar = 1;
        ctx_gpu->color_params[i].Br = 0;
        ctx_gpu->color_params[i].Ag = 1;
        ctx_gpu->color_params[i].Bg = 0;
        ctx_gpu->color_params[i].Ab = 1;
        ctx_gpu->color_params[i].Bb = 0;
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


__global__ void overlap_removal_kernel(Context_gpu* ctx_gpu, Point3fRGB* left, float* depth_left, Point3fRGB* right, float* depth_right, int idx, float *depth_out) {
    int x_idx = threadIdx.x + blockIdx.x * blockDim.x;
    int y_idx = threadIdx.y + blockIdx.y * blockDim.y;
    int pix_idx = x_idx + y_idx * blockDim.x * gridDim.x;
    depth_out[pix_idx] = 128;
    ctx_gpu->mask[pix_idx] = 0;

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

    bool b_overlap = false;
    int index;

    if(x >= 0 && x < Width_depth_HR && y >= 0 && y < Height_depth_HR) {
        for(int i=0; i<patch_size; i++) {
            for(int j=0; j<patch_size; j++) {
                index = ((y - patch_size/2) + i) * Width_depth_HR + (x - patch_size/2) + j;
                if(index >= 0 && index < Width_depth_HR * Height_depth_HR && depth_right[index] > 0 && abs(tempPoint[2] * 1000 - depth_right[index]) < 20) {
                    b_overlap = true;
                }
            }
        }
    }

    if(b_overlap) {
        ctx_gpu->mask[pix_idx] = 1;
    }

    return;
}


__global__ void patch_based_removal_kernel(int *mask, Point3fRGB* verts, float *depth) {
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;

    int pix_idx = x + y * blockDim.x * gridDim.x;
    int index;
    int cnt = 0;

    for(int i=0; i<removal_det_region; i++) {
        for(int j=0; j<removal_det_region; j++) {
            index = ((y - patch_size/2) + i) * Width_depth_HR + (x - patch_size/2) + j;
            if(mask[index] == 1) {
                cnt++;
            }
        }
    }

    if(cnt > removal_det_K) {
        verts[pix_idx].X = 0;
        verts[pix_idx].Y = 0;
        verts[pix_idx].Z = 0;
        // verts[pix_idx].R = 255;
        // verts[pix_idx].G = 0;
        // verts[pix_idx].B = 0;
        depth[pix_idx] = 0;
    }
    return;
}


__global__ void SDC_filter(Context_gpu *ctx_gpu, Point3fRGB* verts, float *depth) {
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int pix_idx = x + y * blockDim.x * gridDim.x;

    if(verts[pix_idx].Z <= z_bbox_min || verts[pix_idx].Z >= z_bbox_max ||
       verts[pix_idx].X <= x_bbox_min || verts[pix_idx].X >= x_bbox_max ||
       verts[pix_idx].Y <= y_bbox_min || verts[pix_idx].Y >= y_bbox_max) {
        return;
    }

    if(x == 0 || y == 0 || x == ctx_gpu->width - 1 || y == ctx_gpu->height - 1) {
        return;
    }

    int c = pix_idx; // center
    int l = pix_idx - 1; // left
    int r = pix_idx + 1; // right
    int t = pix_idx - ctx_gpu->width; // top
    int d = pix_idx + ctx_gpu->width; // down

    if(abs(verts[c].X - verts[l].X) + abs(verts[c].Y - verts[l].Y) + abs(verts[c].Z - verts[l].Z) < SDC_filter_threashold && 
       abs(verts[c].X - verts[r].X) + abs(verts[c].Y - verts[r].Y) + abs(verts[c].Z - verts[r].Z) < SDC_filter_threashold && 
       abs(verts[c].X - verts[t].X) + abs(verts[c].Y - verts[t].Y) + abs(verts[c].Z - verts[t].Z) < SDC_filter_threashold && 
       abs(verts[c].X - verts[d].X) + abs(verts[c].Y - verts[d].Y) + abs(verts[c].Z - verts[d].Z) < SDC_filter_threashold ) {
    }
    else {
        verts[pix_idx].X = 0;
        verts[pix_idx].Y = 0;
        verts[pix_idx].Z = 0;
        verts[pix_idx].R = 0;
        verts[pix_idx].G = 0;
        verts[pix_idx].B = 0;
        depth[pix_idx] = 0;
    }
}


__global__ void isolate_points_filter(Context_gpu *ctx_gpu, Point3fRGB* verts, float *depth) {
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int pix_idx = x + y * blockDim.x * gridDim.x;

    if(verts[pix_idx].Z <= z_bbox_min || verts[pix_idx].Z >= z_bbox_max ||
       verts[pix_idx].X <= x_bbox_min || verts[pix_idx].X >= x_bbox_max ||
       verts[pix_idx].Y <= y_bbox_min || verts[pix_idx].Y >= y_bbox_max) {
        return;
    }

    int index;
    int cnt = 0;
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            index = ((y - 1) + i) * Width_depth_HR + (x - 1) + j;
            if(index > 0 && index < Width_depth_HR * Height_depth_HR) {
                if(depth[index] == 0) {
                    cnt ++;
                }
            }
        }
    }

    if(cnt >= 3) {
        verts[pix_idx].X = 0;
        verts[pix_idx].Y = 0;
        verts[pix_idx].Z = 0;
        verts[pix_idx].R = 0;
        verts[pix_idx].G = 0;
        verts[pix_idx].B = 0;
        depth[pix_idx] = 0;
    }
}


__global__ void color_correction(Context_gpu *ctx_gpu, Point3fRGB* verts, int idx) {
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int pix_idx = x + y * blockDim.x * gridDim.x;

    verts[pix_idx].R = ctx_gpu->color_params[idx].Ar * verts[pix_idx].R + ctx_gpu->color_params[idx].Br;
    verts[pix_idx].G = ctx_gpu->color_params[idx].Ag * verts[pix_idx].G + ctx_gpu->color_params[idx].Bg;
    verts[pix_idx].B = ctx_gpu->color_params[idx].Ab * verts[pix_idx].B + ctx_gpu->color_params[idx].Bb;
} 


void overlap_removal_cuda(Context_gpu* ctx_gpu, framePacket** frameList, float* dpeth_out, Context *ctx_cpu) {
#if 0
    cudaEvent_t start, end;
    cudaEventCreate(&start);
    cudaEventCreate(&end);

    cudaEventRecord(start);
#endif
    dim3 blocks(Width_depth_HR / 32, Height_depth_HR / 8);
    dim3 threads(32, 8);

    for(int i=0; i<numKinects; i++) {
        cudaMemcpy(ctx_gpu->vertices[i], frameList[i]->vertices, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
        cudaMemcpy(ctx_gpu->depth[i], frameList[i]->data_d, sizeof(float) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
    }

    if(ctx_cpu->b_SDC_filter) {
        for(int i=0; i<numKinects; i++) {
            SDC_filter<<<blocks, threads>>>(ctx_gpu, ctx_gpu->vertices[i], ctx_gpu->depth[i]);
        }
    }

    if(ctx_cpu->b_isolate_filter) {
        for(int i=0; i<numKinects; i++) {
            isolate_points_filter<<<blocks, threads>>>(ctx_gpu, ctx_gpu->vertices[i], ctx_gpu->depth[i]);
        }
    }

    int idx;
    for(int i=0; i<numKinects; i++) {
        idx = (i + 1) % numKinects;
        overlap_removal_kernel<<<blocks, threads>>>(ctx_gpu, ctx_gpu->vertices[i], ctx_gpu->depth[i], ctx_gpu->vertices[idx], ctx_gpu->depth[idx], idx, ctx_gpu->depth_out);
        patch_based_removal_kernel<<<blocks, threads>>>(ctx_gpu->mask, ctx_gpu->vertices[i], ctx_gpu->depth[i]);
    }
    for(int i=0; i<numKinects; i++) {
        cudaMemcpy(frameList[i]->vertices, ctx_gpu->vertices[i], sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyDeviceToHost);
    }
    cudaMemcpy(dpeth_out, ctx_gpu->depth_out, sizeof(float) * 512 * 424, cudaMemcpyDeviceToHost);

#if 0
    cudaEventRecord(end);
    cudaEventSynchronize(end);
    float millisecond = 0;
    cudaEventElapsedTime(&millisecond, start, end);
    printf("\t overlap_removal_cuda time = %f ms\n", millisecond);
#endif
}