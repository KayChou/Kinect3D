#include "RGBD_FIFO_Process.h"

std::thread *RGBDProcessThreadTask;
RGBD_FIFO_Process **rgbdProcess;


//=======================================================================================
// create thread which 
//     get data from RGBD FIFO
//     perform calibration if needed
//=======================================================================================
void startAll_RGBD_FIFO_Process(FIFO<framePacket>** input, FIFO<framePacket> **output, bool &calibrationFlag){
    RGBDProcessThreadTask = new std::thread[numKinects];
    rgbdProcess = new RGBD_FIFO_Process*[numKinects];
    //bool *flag = new bool[numKinects];

    for(int i=0; i<numKinects; i++){
        rgbdProcess[i] = new RGBD_FIFO_Process(input[i], output[i]);
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
        if(!*calibrationFlag){
            if(this->output_ != NULL){
                framePacket* newPacket = new framePacket(packet);
                this->output_->put(newPacket);
            }
            if(calibrate.calibrated){ calibrate.calibrated = false; calibrate.calibratedNum = 0; }
        }
         // else perform calibration util success
        else{
            calibrate.performCalibration(packet);
            if(this->output_ != NULL){
                framePacket* newPacket = new framePacket(packet);
                this->output_->put(newPacket);
            }
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


RGBD_FIFO_Process::RGBD_FIFO_Process(FIFO<framePacket>* input, FIFO<framePacket>* output){
    this->input_ = input;
    this->output_ = output;
    finishFlag = false;
}


RGBD_FIFO_Process::~RGBD_FIFO_Process(){

}