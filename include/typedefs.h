#pragma once
#include <stdint.h>
#include <vector>

typedef struct Context {
	bool b_save2Local;
	bool b_start_Camera;
	bool b_Calibration;
	bool b_hasBeenCalibrated;
	bool b_Refine;
	bool b_enableRGB;
	bool b_enableDepth;

	Context(){
		this->b_save2Local = false;
		this->b_start_Camera = false;
		this->b_Calibration = false;
		this->b_hasBeenCalibrated = false;
		this->b_enableRGB = true;
		this->b_enableDepth = true;
	}
} Context;


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


struct TriIndex {
	TriIndex() {
		this->v1 = 0;
		this->v2 = 0;
		this->v3 = 0;
	}
	int v1, v2, v3;
};


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