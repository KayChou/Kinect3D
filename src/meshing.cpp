#include "meshing.h"


void Meshing::init(FIFO<framePacket>* input, FIFO<frameMesh>* output)
{
    this->input = input;
    this->output = output;
}


void Meshing::Loop(Context *context)
{
    int Vn, Fn;
    int ptr_idx = 0;
    Point3f tempPoint;
    triIndex tempTri;

    Point3fRGB *points = new Point3fRGB[context->depth_w * context->depth_h * 5];
    triIndex *triangles = new triIndex[context->depth_w * context->depth_h * 5 * 3];
    int *mask = new int[context->depth_w * context->depth_h]; // revcord valid point index

    Point3fRGB *verts;
    timeval t_start, t_end;
    float t_delay;

    while(true) {
        framePacket *packet = input->get();
#ifdef LOG
        std::printf("meshing get one frame\n");
#endif
        gettimeofday(&t_start, NULL);

        Vn = 0;
        Fn = 0;

        verts = packet->vertices;

        for(int i=0; i<packet->height_d; i++) {
            for(int j=0; j<packet->width_d; j++) {

                ptr_idx = i * packet->width_d + j;

                tempPoint.X = packet->vertices[ptr_idx].X;
                tempPoint.Y = packet->vertices[ptr_idx].Y;
                tempPoint.Z = packet->vertices[ptr_idx].Z;

                // push valid point to "points"
                if(tempPoint.Z > z_bbox_min && tempPoint.Z < z_bbox_max &&
                    tempPoint.X > x_bbox_min && tempPoint.X < x_bbox_max &&
                    tempPoint.Y > y_bbox_min && tempPoint.Y < y_bbox_max) {
                    points[Vn++] = packet->vertices[ptr_idx];
                }
            }
        }
        gettimeofday(&t_end, NULL);
        t_delay = get_time_diff_ms(t_start, t_end);
#ifdef LOG
        std::printf("meshing one frame: %f\n", t_delay); fflush(stdout);
#endif

        if(this->output != NULL) {
            frameMesh *newMesh = new frameMesh();
            newMesh->init(points, triangles, Vn, Fn);
            this->output->put(newMesh);
        }

        packet->destroy();
        usleep(1000);
    }
    delete [] points;
    delete [] triangles;
}