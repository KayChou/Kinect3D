#ifndef KINECT_CAPTURE_H
#define KINECT_CAPTURE_H

#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include "alignment.h"
#include "common.h"


bool openAllKinect(FIFO<framePacket>** output);
void destoryAllKinect();


class Kinect
{
public:
    bool init(std::string serial, FIFO<framePacket>* output, Context *context);
    bool getFrameLoop();
    void setStartFlag(bool flag);
    bool getFinishFlag();

public:
    libfreenect2::Frame *color;
    libfreenect2::Frame *depth;
    FIFO<framePacket>* output;

public:
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::PacketPipeline *pipeline;

    std::string serial;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frames;

    libfreenect2::Freenect2Device::Config config;
    libfreenect2::Registration* registration;

    Context *context;
    bool cameraStarted;
};


class KinectsManager
{
public:
    FIFO<framePacket>** output;
    Context *context;

    libfreenect2::Freenect2 freenect2;
    std::string serials[numKinects];
    Kinect cameras[numKinects];

    std::thread capture_thread[numKinects];

    bool cameraStarted;

public:
    void init(FIFO<framePacket>** output, Context *context);
    bool init_Kinect();
    void loop();
};

#endif