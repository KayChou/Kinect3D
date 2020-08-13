#include "FramePacket.h"

framePacket::framePacket(){ }


void framePacket::init(libfreenect2::Frame *color, 
                       libfreenect2::Frame *depth, 
                       Point3fRGB *verts,
                       int width_c, 
                       int height_c,
                       int width_d,
                       int height_d)
{
    this->width_c = width_c;
    this->height_c = height_c;
    this->width_d = width_d;
    this->height_d = height_d;

    this->data_c = new unsigned char[4 * this->width_c * this->height_c];
    this->data_d = new float[this->width_d * this->height_d];

    this->timestamp_c = color->timestamp;
    this->timestamp_d = depth->timestamp;

    memcpy(this->data_c, color->data, 4 * this->width_c * this->height_c);
    memcpy(this->data_d, depth->data, sizeof(float) * this->width_d * this->height_d);
    this->vertices = verts;
}


void framePacket::destroy(){
    delete [] this->data_c;
    delete [] this->data_d;
    delete [] this->vertices;
}