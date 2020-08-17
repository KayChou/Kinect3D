#include "RGBD_FIFO_Process.h"

std::thread *RGBDProcessThreadTask;
RGBD_FIFO_Process **rgbdProcess;


//=======================================================================================
// create thread which 
//     get data from RGBD FIFO
//     perform calibration if needed
//=======================================================================================
void startAll_RGBD_FIFO_Process(FIFO<framePacket>** input, FIFO<framePacket> **output_pcd, FIFO<framePacket> **output_qt, bool &calibrationFlag){
    RGBDProcessThreadTask = new std::thread[numKinects];
    rgbdProcess = new RGBD_FIFO_Process*[numKinects];

    for(int i=0; i<numKinects; i++){
        rgbdProcess[i] = new RGBD_FIFO_Process(input[i], output_qt[i]);
        RGBDProcessThreadTask[i] = std::thread(&RGBD_FIFO_Process::process, rgbdProcess[i], &calibrationFlag);
    }
    for(int i=0; i<numKinects; i++){
        RGBDProcessThreadTask[i].detach();
    }
}


//=======================================================================================
// wait util class "RGBD_FIFO_Process" finish
// then destruct
//=======================================================================================
void destroyAll_RGBD_FIFO_Process(){
    for(int i=0; i<numKinects; i++){
        while(rgbdProcess[i]->getFinishFlag()){
            delete rgbdProcess[i];
        }
    }
    delete rgbdProcess;
}


//=======================================================================================
// main loop: get data from RGBD FIFO
//      if there is no packet for 5s, break
//      if output is not NULL, copy current packet to output FIFO, for QT render
//      if calibrate button is clicked, perform calibration
//=======================================================================================
void RGBD_FIFO_Process::process(bool* calibrationFlag){
    while(true){
        framePacket *packet = input_->get();
        if( packet == NULL ) { break; } // exit over time(5s)
        // if calibrationFlag = false(stop calibration), just get one packet and display
        //std::printf("process one frame, FIFO len: %d timeStamp: %u \n", input_->cnt, packet->timestamp_c);fflush(stdout);
        
        if(!*calibrationFlag){
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
    std::printf("Thread RGBD_FIFO_Process quit \n", input_->cnt); fflush(stdout);
    this->finishFlag = true;
}


bool RGBD_FIFO_Process::getFinishFlag(){
    return this->finishFlag;
}


void RGBD_FIFO_Process::destory(){

}


RGBD_FIFO_Process::RGBD_FIFO_Process(FIFO<framePacket>* input, FIFO<framePacket>* output_pcd, FIFO<framePacket>* output_qt){
    this->input_ = input;
    this->output_qt = output_qt;
    this->output_pcd = output_pcd;
    finishFlag = false;
}


RGBD_FIFO_Process::~RGBD_FIFO_Process(){

}