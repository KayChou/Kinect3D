#pragma once
#include <stdint.h>
#include <vector>

#define numKinects 3

typedef struct Cam_K {
	float fx; ///< Focal length x (pixel)
    float fy; ///< Focal length y (pixel)
    float cx; ///< Principal point x (pixel)
    float cy; ///< Principal point y (pixel)
} Cam_K;


typedef struct Point3f {
	Point3f(){
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
	}
	Point3f(float X, float Y, float Z){
		this->X = X;
		this->Y = Y;
		this->Z = Z;
	}
	float X;
	float Y;
	float Z;
} Point3f;


typedef struct Point3fRGB { 
    Point3fRGB() {
        this->X = 0;
        this->Y = 0;
        this->Z = 0;
        this->R = 0;
        this->G = 0;
        this->B = 0;
    }
    float X, Y, Z;
    uint8_t R, G, B;
} Point3fRGB;


typedef struct triIndex
{
	triIndex() {
		this->v1 = 0;
		this->v2 = 0;
		this->v3 = 0;
	}
	int v1;
	int v2;
	int v3;
} triIndex;


typedef struct RGB {
    RGB() {
        this->R = 0;
        this->G = 0;
        this->B = 0;
    }
	unsigned char R, G, B;
} RGB;


enum PLYFormat {
    BINARY_BIG_ENDIAN_1,
    BINARY_LITTLE_ENDIAN_1,
    ASCII_1
};


typedef struct Point2f
{
	Point2f()
	{
		this->X = 0;
		this->Y = 0;
	}
	Point2f(float X, float Y)
	{
		this->X = X;
		this->Y = Y;
	}
	float X;
	float Y;
} Point2f;


typedef struct MarkerStruct
{
	int id;
	std::vector<Point2f> corners;
	std::vector<Point3f> points;

	MarkerStruct(){
		id = -1;
	}

	MarkerStruct(int id, std::vector<Point2f> corners, std::vector<Point3f> points){
		this->id = id;

		this->corners = corners;
		this->points = points;
	}
} MarkerInfo;


typedef struct TransformStruct
{
	Point3fRGB *dev_vertices;
    float *dev_R;
    float *dev_T;
} TransformStruct;