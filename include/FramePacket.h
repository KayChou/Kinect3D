#ifndef FRAMEPACKET_H_
#define FRAMEPACKET_H_

#include <stdlib.h>
#include <string.h>
#include <libfreenect2/frame_listener_impl.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include "typedefs.h"

class framePacket{

public:
    size_t width_c;
    size_t width_d;
    size_t height_c;
    size_t height_d;
    uint32_t timestamp_c;
    uint32_t timestamp_d;

    unsigned char* data_c;
    float* data_d;
    Point3fRGB *vertices;

public:
    framePacket();
    void init(libfreenect2::Frame *color, 
                libfreenect2::Frame *depth, 
                Point3fRGB* verts,
                int width_c=1920, 
                int height_c=1080,
                int width_d=512,
                int height_d=424);
    void destroy();
};




#endif