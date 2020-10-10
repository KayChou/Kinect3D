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

	int depth_w;
	int depth_h;

	float Td;

	Context(){
		this->b_save2Local = false;
		this->b_start_Camera = false;
		this->b_Calibration = false;
		this->b_hasBeenCalibrated = false;
		this->b_enableRGB = true;
		this->b_enableDepth = true;
		this->depth_w = 512;
		this->depth_h = 424;
		this->Td = 3.0f;
	}
} Context;