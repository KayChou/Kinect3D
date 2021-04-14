#include "cuda.cuh"


__device__ void RotatePoint(float *point, float* R, float* T)
{
    if(point[0]==0 && point[1]==0 && point[2]==0) {
        return;
    }
	float res[3];
	point[0] += T[0];
	point[1] += T[1];
	point[2] += T[2];

	res[0] = point[0] * R[0] + point[1] * R[1] + point[2] * R[2];
	res[1] = point[0] * R[3] + point[1] * R[4] + point[2] * R[5];
	res[2] = point[0] * R[6] + point[1] * R[7] + point[2] * R[8];

	point[0] = res[0];
	point[1] = res[1];
	point[2] = res[2];
}


__global__ void Transform_Kernel(int width_d, int height_d, Point3fRGB *vertices, float *R, float* T)
{
    float tempPoint[3];

    int j = threadIdx.x + blockIdx.x * blockDim.x;
    int i = threadIdx.y + blockIdx.y * blockDim.y;
    int ptr_idx = i*width_d + j;

    tempPoint[0] = vertices[ptr_idx].X;
    tempPoint[1] = vertices[ptr_idx].Y;
    tempPoint[2] = vertices[ptr_idx].Z;

    RotatePoint(tempPoint, R, T);

    vertices[ptr_idx].X = tempPoint[0];
    vertices[ptr_idx].Y = tempPoint[1];
    vertices[ptr_idx].Z = tempPoint[2];
}


TransformStruct* Transform_gpu_init()
{
    TransformStruct *transformStruct;
    cudaMallocManaged((void**)&transformStruct, sizeof(TransformStruct));
    cudaMalloc((void**)&transformStruct->dev_vertices, sizeof(Point3fRGB) * Width_depth_HR * Height_depth_HR);
    cudaMalloc((void**)&transformStruct->dev_R, sizeof(float) * 9);
    cudaMalloc((void**)&transformStruct->dev_T, sizeof(float) * 3);
    return transformStruct;
}


void Transform(int width_d, int height_d, framePacket *packet, std::vector<std::vector<float>> &R, std::vector<float> &T, TransformStruct *transformStruct)
{
#if 0
    cudaEvent_t start, end;
    cudaEventCreate(&start);
    cudaEventCreate(&end);

    cudaEventRecord(start);
#endif
    cudaMemcpy(transformStruct->dev_vertices, packet->vertices, width_d * height_d * sizeof(Point3fRGB), cudaMemcpyHostToDevice );

    float host_R[9];
    float host_T[3];

    host_R[0] = R[0][0];
    host_R[1] = R[0][1];
    host_R[2] = R[0][2];
    host_R[3] = R[1][0];
    host_R[4] = R[1][1];
    host_R[5] = R[1][2];
    host_R[6] = R[2][0];
    host_R[7] = R[2][1];
    host_R[8] = R[2][2];
    host_T[0] = T[0];
    host_T[1] = T[1];
    host_T[2] = T[2];

    cudaMemcpy(transformStruct->dev_R, host_R, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(transformStruct->dev_T, host_T, 3 * sizeof(float), cudaMemcpyHostToDevice);

    dim3 blocks(width_d / 8, height_d / 8);
    dim3 threads(8, 8);

    Transform_Kernel<<<blocks, threads>>>(width_d, height_d, transformStruct->dev_vertices, transformStruct->dev_R, transformStruct->dev_T);

    cudaMemcpy(packet->vertices, transformStruct->dev_vertices, width_d * height_d * sizeof(Point3fRGB), cudaMemcpyDeviceToHost);

#if 0
    cudaEventRecord(end);
    cudaEventSynchronize(end);
    float millisecond = 0;
    cudaEventElapsedTime(&millisecond, start, end);
    printf("\t transform time = %f ms\n", millisecond);
#endif
}
