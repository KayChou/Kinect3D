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

enum systemStatus{
    SYSTEM_RUNNING,
    SYSTEM_PAUSE,
    SYSTEM_FINISH
};