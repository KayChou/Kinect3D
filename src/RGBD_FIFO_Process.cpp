#include "RGBD_FIFO_Process.h"

//=======================================================================================
// main loop: get data from RGBD FIFO
//      if calibrate button is clicked, perform calibration
//=======================================================================================
void RGBD_FIFO_Process::process(Context *context){
    Point3fRGB *points = new Point3fRGB[context->depth_w * context->depth_h * 5];
    triIndex *triangles = new triIndex[context->depth_w * context->depth_h * 5 * 3];
    int *mask = new int[context->depth_w * context->depth_h]; // revcord valid point index

    int Vn = 0;
    int Fn = 0;
    int ptr_idx = 0;

    Point3fRGB *verts;

    while(true){
        framePacket *packet = input->get();
        std::printf("RGBD Process get one frame\n");
        
        if(context->b_Calibration){
            context->b_hasBeenCalibrated = calibrate.performCalibration(packet, T, R);
        }

        Point3f tempPoint;
        triIndex tempTri;
        Vn = 0;
        Fn = 0;

        verts = packet->vertices;

        for(int i=0; i<packet->height_d; i++) {
            for(int j=0; j<packet->width_d; j++) {

                ptr_idx = i*packet->width_d + j;

                tempPoint.X = packet->vertices[ptr_idx].X;
                tempPoint.Y = packet->vertices[ptr_idx].Y;
                tempPoint.Z = packet->vertices[ptr_idx].Z;

                if(context->b_hasBeenCalibrated) {
                    RotatePoint(tempPoint, R, T);
                    packet->vertices[ptr_idx].X = tempPoint.X;
                    packet->vertices[ptr_idx].Y = tempPoint.Y;
                    packet->vertices[ptr_idx].Z = tempPoint.Z;
                }

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

                if(i > 0 && j > 0){
                    int c = ptr_idx; // center
                    int l = ptr_idx - 1; // left
                    int t = ptr_idx - packet->width_d; // top
                    int lt = ptr_idx - 1 - packet->width_d; // left top

                    if(abs(verts[c].X - verts[l].X) + abs(verts[c].Y - verts[l].Y) + abs(verts[c].Z - verts[l].Z) < context->Td && 
                        abs(verts[t].X - verts[l].X) + abs(verts[t].Y - verts[l].Y) + abs(verts[t].Z - verts[l].Z) < context->Td && 
                        abs(verts[c].X - verts[t].X) + abs(verts[c].Y - verts[t].Y) + abs(verts[c].Z - verts[t].Z) < context->Td && 
                        mask[c] && mask[l] && mask[t]) {
                        tempTri.v1 = mask[l];
                        tempTri.v2 = mask[c];
                        tempTri.v3 = mask[t];
                        triangles[Fn++] = tempTri;
                    }

                    if(abs(verts[lt].X - verts[l].X) + abs(verts[lt].Y - verts[l].Y) + abs(verts[lt].Z - verts[l].Z) < context->Td && 
                        abs(verts[t].X - verts[l].X) + abs(verts[t].Y - verts[l].Y) + abs(verts[t].Z - verts[l].Z) < context->Td && 
                        abs(verts[lt].X - verts[t].X) + abs(verts[lt].Y - verts[t].Y) + abs(verts[lt].Z - verts[t].Z) < context->Td &&
                        mask[l] && mask[lt] && mask[t]) {
                        tempTri.v1 = mask[l];
                        tempTri.v2 = mask[lt];
                        tempTri.v3 = mask[t];
                        triangles[Fn++] = tempTri;
                    }
                }
            }
        }
        Vn--;
        Fn--;
        std::printf("Vn: %d Fn %d\n", Vn, Fn); fflush(stdout);
        
        if(this->output_qt != NULL){ // FIFO for QT image render
            framePacket* newPacket = new framePacket(packet);
            this->output_qt->put(newPacket);
        }
        if(this->output_pcd != NULL){ // FIFO for next step(save to local or pointcloud render)
            frameMesh *newMesh = new frameMesh();
            newMesh->init(points, triangles, Vn, Fn);
            this->output_pcd->put(newMesh);
        }
        packet->destroy();
        usleep(1000);
    }
    delete [] points;
    delete [] triangles;
    std::printf("Thread RGBD_FIFO_Process quit \n", input->cnt); fflush(stdout);
}


void RGBD_FIFO_Process::init(FIFO<framePacket>* input, FIFO<frameMesh>* output_pcd, FIFO<framePacket>* output_qt){
    this->input = input;
    this->output_qt = output_qt;
    this->output_pcd = output_pcd;
}


void RGBD_FIFO_Process::RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T)
{
	std::vector<float> res(3);
	point.X += T[0];
	point.Y += T[1];
	point.Z += T[2];

	res[0] = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res[1] = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res[2] = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	point.X = res[0];
	point.Y = res[1];
	point.Z = res[2];
}
