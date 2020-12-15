#include "cuda.cuh"


__device__ void RotatePoint(float *point, float* R, float* T)
{
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


void Transform(int width_d, int height_d, framePacket *packet, std::vector<std::vector<float>> &R, std::vector<float> &T)
{
    Point3fRGB *dev_vertices;
    float *dev_R;
    float *dev_T;

    cudaMalloc((void**)&dev_R, 9 * sizeof(float));
    cudaMalloc((void**)&dev_T, 3 * sizeof(float));
    cudaMalloc((void**)&dev_vertices, width_d * height_d * sizeof(Point3fRGB));
    cudaMemcpy(dev_vertices, packet->vertices, width_d * height_d * sizeof(Point3fRGB), cudaMemcpyHostToDevice );

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

    cudaMemcpy(dev_R, host_R, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(dev_T, host_T, 3 * sizeof(float), cudaMemcpyHostToDevice);

    dim3 blocks(width_d / 8, height_d / 8);
    dim3 threads(8, 8);

    Transform_Kernel<<<blocks, threads>>>(width_d, height_d, dev_vertices, dev_R, dev_T);

    cudaMemcpy(packet->vertices, dev_vertices, width_d * height_d * sizeof(Point3fRGB), cudaMemcpyDeviceToHost );

    cudaFree(dev_vertices);
    cudaFree(dev_R);
    cudaFree(dev_T);
}
