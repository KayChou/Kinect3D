#ifndef KINECT_CAPTURE_H
#define KINECT_CAPTURE_H

#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include "common.h"
#include "FIFO.h"
#include "typedef.h"
#include "utils.h"

#define USE_RAW_DEPTH 1


bool openAllKinect();
void destoryAllKinect();


class Kinect
{
public:
    bool init(int idx, std::string serial, float colorExposure);
    bool getFrameLoop(int *stop_flag, FIFO<RGBD> *output);
    void setStartFlag(bool flag);
    bool getFinishFlag();

public:
    libfreenect2::Frame *color;
    libfreenect2::Frame *depth;

public:
    int idx;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::PacketPipeline *pipeline;

    std::string serial;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frames;

    libfreenect2::Freenect2Device::Config config;
    libfreenect2::Registration* registration;

    bool cameraStarted;
    float colorExposure;
};


class KinectsManager
{
public:
    libfreenect2::Freenect2 freenect2;
    Kinect cameras[CAM_NUM];

    std::string DeviceSerialNumber[4];

    std::thread capture_thread[CAM_NUM];

    bool cameraStarted;
    int *stop_flag;

    FIFO<RGBD> **output;

public:
    void init(int *stop_flag, FIFO<RGBD> **output);
    bool init_Kinect();
    void loop();
};

#endif