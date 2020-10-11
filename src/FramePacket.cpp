#include "FramePacket.h"

framePacket::framePacket(){ }


framePacket::framePacket(framePacket* packet){
    this->width_c = packet->width_c;
    this->height_c = packet->height_c;
    this->width_d = packet->width_d;
    this->height_d = packet->height_d;

    this->data_c = new unsigned char[4 * packet->width_c * packet->height_c];
    this->data_d = new float[packet->width_d * packet->height_d];
    this->vertices = new Point3fRGB[packet->width_d * packet->height_d];

    this->timestamp_c = packet->timestamp_c;
    this->timestamp_d = packet->timestamp_d;

    memcpy(this->data_c, packet->data_c, 4 * this->width_c * this->height_c);
    memcpy(this->data_d, packet->data_d, sizeof(float) * this->width_d * this->height_d);
    memcpy(this->vertices, packet->vertices, sizeof(Point3fRGB) * this->width_d * this->height_d);
}


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


frameMesh::frameMesh() { }


void frameMesh::init(Point3fRGB *vertices, triIndex* triangles, size_t Vn, size_t Fn)
{
    this->Vn = Vn;
    this->Fn = Fn;
    this->vertices = new Point3fRGB[Vn];
    this->triangles = new triIndex[Fn];

    memcpy(this->vertices, vertices, sizeof(Point3fRGB) * this->Vn);
    memcpy(this->triangles, triangles, sizeof(triIndex) * this->Fn);
}

void frameMesh::destroy()
{
    delete [] this->vertices;
    delete [] this->triangles;
}