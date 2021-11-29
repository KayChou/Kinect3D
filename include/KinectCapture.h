#ifndef KINECT_CAPTURE_H
#define KINECT_CAPTURE_H

#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include "common.h"

#define USE_RAW_DEPTH 1


bool openAllKinect();
void destoryAllKinect();


class Kinect
{
public:
    bool init(int idx, std::string serial, float colorExposure);
    bool getFrameLoop(int *stop_flag);
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

    char dataset_path[256];
};


class KinectsManager
{
public:
    libfreenect2::Freenect2 freenect2;
    Kinect cameras[numKinects];

    std::string DeviceSerialNumber[4];

    std::thread capture_thread[numKinects];

    bool cameraStarted;
    int *stop_flag;

public:
    void init(int *stop_flag);
    bool init_Kinect();
    void loop();
};

#endif