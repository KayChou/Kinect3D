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
        // if( packet == NULL ) { break; } // exit over time(5s)
        // if calibrationFlag = false(stop calibration), just get one packet and display
        
        if(!context->b_Calibration){
            if(calibrate.calibrated){ calibrate.calibrated = false; calibrate.calibratedNum = 0; }
        }
        // else perform calibration util success
        else{
            calibrate.performCalibration(packet);
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
    }
    std::printf("Thread RGBD_FIFO_Process quit \n", input->cnt); fflush(stdout);
}


void RGBD_FIFO_Process::init(FIFO<framePacket>* input, FIFO<framePacket>* output_pcd, FIFO<framePacket>* output_qt){
    this->input = input;
    this->output_qt = output_qt;
    this->output_pcd = output_pcd;
}
