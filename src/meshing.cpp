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
                    mask[ptr_idx] = Vn;
                    points[Vn++] = packet->vertices[ptr_idx];
                }
                else{
                    mask[ptr_idx] = 0;
                }

                // if(i > 0 && j > 0){
                //     int c = ptr_idx; // center
                //     int l = ptr_idx - 1; // left
                //     int t = ptr_idx - packet->width_d; // top
                //     int lt = ptr_idx - 1 - packet->width_d; // left top

                //     if(abs(verts[c].X - verts[l].X) + abs(verts[c].Y - verts[l].Y) + abs(verts[c].Z - verts[l].Z) < context->Td && 
                //         abs(verts[t].X - verts[l].X) + abs(verts[t].Y - verts[l].Y) + abs(verts[t].Z - verts[l].Z) < context->Td && 
                //         abs(verts[c].X - verts[t].X) + abs(verts[c].Y - verts[t].Y) + abs(verts[c].Z - verts[t].Z) < context->Td && 
                //         mask[c] && mask[l] && mask[t]) {
                //         tempTri.v1 = mask[l];
                //         tempTri.v2 = mask[c];
                //         tempTri.v3 = mask[t];
                //         triangles[Fn++] = tempTri;
                //     }

                //     if(abs(verts[lt].X - verts[l].X) + abs(verts[lt].Y - verts[l].Y) + abs(verts[lt].Z - verts[l].Z) < context->Td && 
                //         abs(verts[t].X - verts[l].X) + abs(verts[t].Y - verts[l].Y) + abs(verts[t].Z - verts[l].Z) < context->Td && 
                //         abs(verts[lt].X - verts[t].X) + abs(verts[lt].Y - verts[t].Y) + abs(verts[lt].Z - verts[t].Z) < context->Td &&
                //         mask[l] && mask[lt] && mask[t]) {
                //         tempTri.v1 = mask[l];
                //         tempTri.v2 = mask[lt];
                //         tempTri.v3 = mask[t];
                //         triangles[Fn++] = tempTri;
                //     }
                // }
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