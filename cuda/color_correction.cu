#include "cuda.cuh"


extern __device__ void backforward_map(float *point, float *R, float *T)
{
    float res[3];
    res[0] = point[0] * R[0] + point[1] * R[1] + point[2] * R[2];
    res[1] = point[0] * R[3] + point[1] * R[4] + point[2] * R[5];
    res[2] = point[0] * R[6] + point[1] * R[7] + point[2] * R[8];

    point[0] = res[0] - T[0];
    point[1] = res[1] - T[1];
    point[2] = res[2] - T[2];
}


__global__ void get_matched_points_kernel(Context_gpu* ctx_gpu, Point3fRGB* left, Point3fRGB* right, float *depth_right, int idx) {
    int x_idx = threadIdx.x + blockIdx.x * blockDim.x;
    int y_idx = threadIdx.y + blockIdx.y * blockDim.y;
    int pix_idx = x_idx + y_idx * blockDim.x * gridDim.x;

    float tempPoint[3];
    tempPoint[0] = left[pix_idx].X;
    tempPoint[1] = left[pix_idx].Y;
    tempPoint[2] = left[pix_idx].Z;

    backforward_map(tempPoint, ctx_gpu->invR[idx], ctx_gpu->T[idx]);

    if(tempPoint[2] <= 0) {
        return;
    }

    int x = (int)(tempPoint[0] * ctx_gpu->x_ratio / tempPoint[2] * ctx_gpu->K[idx].fx + ctx_gpu->K[idx].cx - 0.5);
    int y = (int)(tempPoint[1] * ctx_gpu->y_ratio / tempPoint[2] * ctx_gpu->K[idx].fy + ctx_gpu->K[idx].cy - 0.5);

    int index;

    if(x >= 0 && x < Width_depth_HR && y >= 0 && y < Height_depth_HR) {
        for(int i=0; i<patch_size; i++) {
            for(int j=0; j<patch_size; j++) {
                index = ((y - patch_size/2) + i) * Width_depth_HR + (x - patch_size/2) + j;
                if(index >= 0 && index < Width_depth_HR * Height_depth_HR && depth_right[index] > 0 && abs(tempPoint[2] * 1000 - depth_right[index]) < 20) {
                    ctx_gpu->matched_idx[pix_idx] = y * Width_depth_HR + x;
                }
            }
        }
    }

    return;
}


void get_matched_points_cuda(Context_gpu* ctx_gpu, Point3fRGB *verts1, Point3fRGB *verts2, float *depth_right, int cam_idx) {
    cudaMemcpy(ctx_gpu->verts1_match, verts1, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
    cudaMemcpy(ctx_gpu->verts2_match, verts2, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
    cudaMemcpy(ctx_gpu->depth_match, depth_right, sizeof(float) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);

    dim3 blocks(Width_depth_HR / 32, Height_depth_HR / 8);
    dim3 threads(32, 8);

    get_matched_points_kernel<<<blocks, threads>>>(ctx_gpu, ctx_gpu->verts1_match, ctx_gpu->verts2_match, ctx_gpu->depth_match, cam_idx);

    int idx = 0;
    FILE *f = fopen("match_points_warp.csv", "w");
    for(int j=0; j<Width_depth_HR * Height_depth_HR; j++) {
        idx = ctx_gpu->matched_idx[j];
        if(idx > 0 && idx < Width_depth_HR * Height_depth_HR) {
            fprintf(f, "%f,%f,%f,%f,%f,%f,%u,%u,%u,%u,%u,%u\n", ctx_gpu->verts1_match[j].X, ctx_gpu->verts1_match[j].Y, ctx_gpu->verts1_match[j].Z, 
                                                                ctx_gpu->verts2_match[idx].X, ctx_gpu->verts2_match[idx].Y, ctx_gpu->verts2_match[idx].Z,
                                                                ctx_gpu->verts1_match[j].R, ctx_gpu->verts1_match[j].G, ctx_gpu->verts1_match[j].B, 
                                                                ctx_gpu->verts2_match[idx].R, ctx_gpu->verts2_match[idx].G, ctx_gpu->verts2_match[idx].B);
        }
    }
    fclose(f);
}