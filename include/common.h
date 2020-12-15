#pragma once

#include <string>
#include <sys/time.h>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv/cv.h"

#include <QWidget>
#include <vector>

#include "PlyIO.h"

#define framemax 300
#define FIFO_LEN 30
#define numKinects 1
#define typesDefault libfreenect2::Frame::Color | libfreenect2::Frame::Depth

#define x_bbox_min -1
#define x_bbox_max 1
#define y_bbox_min -1
#define y_bbox_max 1
#define z_bbox_min -1
#define z_bbox_max 1


typedef struct Context {
	bool b_save2Local;
	bool b_start_Camera;
	bool b_Calibration;
	bool b_hasBeenCalibrated;
	bool b_Refine;
	bool b_enableRGB;
	bool b_enableDepth;
	bool b_cfg_saved;

	int depth_w;
	int depth_h;

	float Td;
	std::string DeviceSerialNumber[4];

	Cam_K K[numKinects];
	std::vector<float> T[numKinects];
	std::vector<std::vector<float>> R[numKinects];

	Context(){
		this->b_save2Local = false;
		this->b_start_Camera = false;
		this->b_Calibration = false;
		this->b_hasBeenCalibrated = false;
		this->b_cfg_saved = false;
		this->b_enableRGB = true;
		this->b_enableDepth = true;
		this->depth_w = Width_depth_HR;
		this->depth_h = Height_depth_HR;
		this->Td = 0.025f;

		DeviceSerialNumber[0] = "021871240647";
		DeviceSerialNumber[1] = "010845342847";

		for(int n=0; n<numKinects; n++) {
			this->R[n].resize(3);

			for (int i = 0; i < 3; i++) {
				this->R[n][i].resize(3);
				for (int j = 0; j < 3; j++) {
					this->R[n][i][j] = 0;
				}
			}
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
