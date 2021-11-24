#ifndef FRAMEPACKET_H_
#define FRAMEPACKET_H_

#include <stdlib.h>
#include <string.h>
#include <libfreenect2/frame_listener_impl.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include "typedefs.h"

#define Width_depth_HR 512
#define Height_depth_HR 424

#if 0
#define x_bbox_min -0.5
#define x_bbox_max 0.5
#define y_bbox_min -0.5
#define y_bbox_max 0.5
#define z_bbox_min -0.5
#define z_bbox_max 0.5
#endif

#if 1
#define x_bbox_min -0.75
#define x_bbox_max 0.75
#define y_bbox_min -0.75
#define y_bbox_max 0.75
#define z_bbox_min -0.75
#define z_bbox_max 0.75
#endif

#if 0
#define x_bbox_min -1
#define x_bbox_max 1
#define y_bbox_min -1
#define y_bbox_max 1
#define z_bbox_min -1
#define z_bbox_max 1
#endif

class framePacket{

public:
    size_t width_c;
    size_t width_d;
    size_t height_c;
    size_t height_d;
    uint32_t timestamp_c;
    uint32_t timestamp_d;

    unsigned char* data_c; // color image
    float* data_d; // depth image
    Point3fRGB *vertices; // point cloud verts(X Y Z R G B)

public:
    framePacket();
    framePacket(framePacket* packet); // copy construction
    void init(libfreenect2::Frame *color, 
                libfreenect2::Frame *depth, 
                Point3fRGB* verts,
                int width_c=1920, 
                int height_c=1080,
                int width_d=Width_depth_HR,
                int height_d=Height_depth_HR);
    void destroy();
};


class frameMesh
{
public:
    size_t Vn;
    size_t Fn;
    Point3fRGB *vertices;
    triIndex* triangles;

public:
    frameMesh();
    void init(Point3fRGB *vertices, triIndex* triangles, size_t Vn, size_t Fn);

    void destroy();
};


typedef struct Context {
	bool b_save2Local;
	bool b_start_Camera;
	bool b_Calibration;
	bool b_all_sensor_calibrated;
	bool b_Refine;
	bool b_enableRGB;
	bool b_enableDepth;
	bool b_cfg_saved;
	bool b_SDC_filter;
	bool b_isolate_filter;

	int depth_w;
	int depth_h;
	int calibration_idx;

	float Td;
	float x_ratio; // super-resolution ratio: HR_width / 512
	float y_ratio;
	std::string DeviceSerialNumber[4];

	bool b_hasBeenCalibrated[numKinects];
	bool b_refined_data_ready[numKinects];
	Cam_K K[numKinects];
	std::vector<float> T[numKinects];
	std::vector<std::vector<float>> R[numKinects];
	std::vector<std::vector<float>> invR[numKinects];
	framePacket frame_to_be_refined[numKinects];
	float colorExposure[numKinects];

	Context(){
		this->b_save2Local = false;
		this->b_start_Camera = false;
		this->b_Calibration = false;
		this->b_all_sensor_calibrated = false;
		this->b_cfg_saved = false;
		this->b_enableRGB = true;
		this->b_Refine = false;
		this->b_enableDepth = true;
		this->b_SDC_filter = true;
		this->b_isolate_filter = true;
		this->depth_w = Width_depth_HR;
		this->depth_h = Height_depth_HR;
		this->Td = 0.025f;
		this->x_ratio = (float)(512 - 1) / (float)Width_depth_HR;
		this->y_ratio = (float)(424 - 1) / (float)Height_depth_HR;;

		DeviceSerialNumber[0] = "036669543547";
		DeviceSerialNumber[1] = "021871240647";
		DeviceSerialNumber[2] = "005275250347";
		//DeviceSerialNumber[1] = "010845342847";

		colorExposure[0] = 0.0;
		colorExposure[1] = 0.0;
		calibration_idx = 0;

		for(int n=0; n<numKinects; n++) {
			this->b_hasBeenCalibrated[n] = false;
			this->b_refined_data_ready[n] = false;
			this->frame_to_be_refined[n].vertices = new Point3fRGB[Width_depth_HR * Height_depth_HR];
			this->frame_to_be_refined[n].data_d = new float[Width_depth_HR * Height_depth_HR];
			this->R[n].resize(3);
			this->invR[n].resize(3);

			for (int i = 0; i < 3; i++) {
				this->R[n][i].resize(3);
				this->invR[n][i].resize(3);
				for (int j = 0; j < 3; j++) {
					this->R[n][i][j] = 0;
					this->invR[n][i][j] = 0;
				}
			}
			this->R[n][0][0] = 1;
			this->R[n][1][1] = 1;
			this->R[n][2][2] = 1;
			this->T[n].resize(3);
			this->T[n][0] = 0;
			this->T[n][1] = 0;
			this->T[n][2] = 0;
			this->K[n].fx = 0;
			this->K[n].fy = 0;
			this->K[n].cx = 0;
			this->K[n].cx = 0;
		}
	}
} Context;


typedef struct Context_gpu {
	float* R[numKinects];
	float* T[numKinects];
	float* invR[numKinects];

	Cam_K K[numKinects];

	int width;
	int height;

	float x_ratio;
	float y_ratio;

	Point3fRGB* vertices[numKinects];
	float *depth[numKinects];
	float *depth_out;
	int *mask;

	// datas using for coloe correction
	int *matched_idx;
	Point3fRGB *verts1_match;
	Point3fRGB *verts2_match;
	float *depth_match;

	bool *overlap_removal;

	color_correct_params color_params[numKinects];
} Context_gpu;

#endif
