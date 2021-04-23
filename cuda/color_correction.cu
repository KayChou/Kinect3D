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


__device__ void rgb2yuv(uint8_t r, uint8_t g, uint8_t b, uint8_t &y, uint8_t &u, uint8_t &v) {

    float Y, U, V;
    Y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
	U = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
	V = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;

    y = (Y >= 0) ? ((Y <= 255) ? Y : 255) : 0;
    u = (U >= 0) ? ((U <= 255) ? U : 255) : 0;
    v = (V >= 0) ? ((V <= 255) ? V : 255) : 0;
}


__device__ void yuv2rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
    float R, G, B;
    B = (298 * (y - 16) + 516 * (u - 128)) >> 8;
    G = (298 * (y - 16) - 208 * (v - 128) - 100 * (u - 128)) >> 8;
    R = (298 * (y - 16) + 408 * (v - 128)) >> 8;

    r = (R >= 0) ? ((R <= 255) ? R : 255) : 0;
    g = (G >= 0) ? ((G <= 255) ? G : 255) : 0;
    b = (B >= 0) ? ((B <= 255) ? B : 255) : 0;
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


void get_matched_points_cuda(Context_gpu* ctx_gpu, Point3fRGB *verts1, Point3fRGB *verts2, float *depth_right, int cam_idx, 
                             std::vector<Point> &data_r, std::vector<Point> &data_g, std::vector<Point> &data_b) 
{
    cudaMemcpy(ctx_gpu->verts1_match, verts1, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
    cudaMemcpy(ctx_gpu->verts2_match, verts2, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);
    cudaMemcpy(ctx_gpu->depth_match, depth_right, sizeof(float) * Width_depth_HR * Height_depth_HR, cudaMemcpyHostToDevice);

    dim3 blocks(Width_depth_HR / 32, Height_depth_HR / 8);
    dim3 threads(32, 8);

    get_matched_points_kernel<<<blocks, threads>>>(ctx_gpu, ctx_gpu->verts1_match, ctx_gpu->verts2_match, ctx_gpu->depth_match, cam_idx);

    int idx = 0;
    FILE *f = fopen("match_points_warp.csv", "w");

    Point temp_R, temp_G, temp_B;
    for(int j=0; j<Width_depth_HR * Height_depth_HR; j++) {
        idx = ctx_gpu->matched_idx[j];
        if(idx > 0 && idx < Width_depth_HR * Height_depth_HR) {
            temp_R.x = (float)ctx_gpu->verts1_match[j].R;
            temp_R.y = (float)ctx_gpu->verts2_match[idx].R;
            temp_G.x = (float)ctx_gpu->verts1_match[j].G;
            temp_G.y = (float)ctx_gpu->verts2_match[idx].G;
            temp_B.x = (float)ctx_gpu->verts1_match[j].B;
            temp_B.y = (float)ctx_gpu->verts2_match[idx].B;

            if(temp_R.x > 0 && temp_R.x < 255 && temp_R.y > 0 && temp_R.y < 255 && 
               temp_G.x > 0 && temp_G.x < 255 && temp_G.y > 0 && temp_G.y < 255 && 
               temp_B.x > 0 && temp_B.x < 255 && temp_B.y > 0 && temp_B.y < 255)// &&
            //    abs(temp_R.x - temp_R.y) < 50  && abs(temp_G.x - temp_G.y) < 50  && abs(temp_B.x - temp_B.y) < 50) 
            {
                fprintf(f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ctx_gpu->verts1_match[j].X, ctx_gpu->verts1_match[j].Y, ctx_gpu->verts1_match[j].Z, 
                                                            ctx_gpu->verts2_match[idx].X, ctx_gpu->verts2_match[idx].Y, ctx_gpu->verts2_match[idx].Z,
                                                            temp_R.x, temp_G.x, temp_B.x, temp_R.y, temp_G.y, temp_B.y);
                data_r.push_back(temp_R);
                data_g.push_back(temp_G);
                data_b.push_back(temp_B);
            }
        }
    }
    fclose(f);

}