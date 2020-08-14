#ifndef KINECT_CAPTURE_H
#define KINECT_CAPTURE_H

#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include "common.h"


bool openAllKinect(int numOfKinects, FIFO<framePacket>** output);
void destoryAllKinect(int numOfKinects);


class oneKinect
{
public:
    oneKinect();
    oneKinect(std::string serial, FIFO<framePacket>* output, int types=typesDefault);
    ~oneKinect();

    bool init(std::string serial, int types=typesDefault);
    bool getFrameLoop();
    void setStartFlag(bool flag);
    bool getFinishFlag();

public:
    libfreenect2::Frame *color_;
    libfreenect2::Frame *depth_;
    FIFO<framePacket>* output_;

public:
    libfreenect2::Freenect2 freenect2_;
    libfreenect2::Freenect2Device *dev_;
    libfreenect2::PacketPipeline *pipeline_;

    std::string serial_;
    int types_;
    libfreenect2::SyncMultiFrameListener *listener_;
    libfreenect2::FrameMap frames_;

    libfreenect2::Freenect2Device::Config config_;
    libfreenect2::Registration* registration_;

    bool startFlag;
    bool finishFlag;
};

#endif