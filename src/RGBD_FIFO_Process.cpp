#include "RGBD_FIFO_Process.h"

//=======================================================================================
// main loop: get data from RGBD FIFO
//      if there is no packet for 5s, break
//      if output is not NULL, copy current packet to output FIFO, for QT render
//      if calibrate button is clicked, perform calibration
//=======================================================================================
void RGBD_FIFO_Process::process(Context *context){
    while(true){
        framePacket *packet = input->get();
        std::printf("RGBD Process get one frame\n");
        
        if(context->b_Calibration){
            context->b_hasBeenCalibrated = calibrate.performCalibration(packet, T, R);
        }

        if(context->b_hasBeenCalibrated) {
            Point3f tempPoint;
            for(int i=0; i<packet->height_d; i++){
                for(int j=0; j<packet->width_d; j++){
                    tempPoint.X = packet->vertices[i*packet->width_d + j].X;
                    tempPoint.Y = packet->vertices[i*packet->width_d + j].Y;
                    tempPoint.Z = packet->vertices[i*packet->width_d + j].Z;

                    RotatePoint(tempPoint, R, T);
                    packet->vertices[i*packet->width_d + j].X = tempPoint.X;
                    packet->vertices[i*packet->width_d + j].Y = tempPoint.Y;
                    packet->vertices[i*packet->width_d + j].Z = tempPoint.Z;
                }
            }
        }
        
        if(this->output_qt != NULL){ // FIFO for QT image render
            framePacket* newPacket = new framePacket(packet);
            this->output_qt->put(newPacket);
        }
        if(this->output_pcd != NULL){ // FIFO for next step(save to local or pointcloud render)
            framePacket* newPacket = new framePacket(packet);
            this->output_pcd->put(newPacket);
        }
        packet->destroy();
        usleep(20000);
    }
    std::printf("Thread RGBD_FIFO_Process quit \n", input->cnt); fflush(stdout);
}


void RGBD_FIFO_Process::init(FIFO<framePacket>* input, FIFO<framePacket>* output_pcd, FIFO<framePacket>* output_qt){
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
