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
#define numKinects 2
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
	bool b_all_sensor_calibrated;
	bool b_Refine;
	bool b_enableRGB;
	bool b_enableDepth;
	bool b_cfg_saved;

	int depth_w;
	int depth_h;

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

	Context(){
		this->b_save2Local = false;
		this->b_start_Camera = false;
		this->b_Calibration = false;
		this->b_all_sensor_calibrated = false;
		this->b_cfg_saved = false;
		this->b_enableRGB = true;
		this->b_Refine = false;
		this->b_enableDepth = true;
		this->depth_w = Width_depth_HR;
		this->depth_h = Height_depth_HR;
		this->Td = 0.025f;
		this->x_ratio = (float)(512 - 1) / (float)Width_depth_HR;
		this->y_ratio = (float)(424 - 1) / (float)Height_depth_HR;;

		DeviceSerialNumber[0] = "021871240647";
		DeviceSerialNumber[1] = "036669543547";
		//DeviceSerialNumber[1] = "010845342847";

		for(int n=0; n<numKinects; n++) {
			this->b_hasBeenCalibrated[n] = false;
			this->b_refined_data_ready[n] = false;
			this->frame_to_be_refined[n].vertices = new Point3fRGB[Width_depth_HR * Height_depth_HR];
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
