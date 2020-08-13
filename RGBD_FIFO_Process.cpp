#include "RGBD_FIFO_Process.h"

std::thread *RGBDProcessThreadTask;
RGBD_FIFO_Process **rgbdProcess;


void startAll_RGBD_FIFO_Process(FIFO** input, FIFO *output, bool &calibrationFlag){
    RGBDProcessThreadTask = new std::thread[numKinects];
    rgbdProcess = new RGBD_FIFO_Process*[numKinects];

    for(int i=0; i<numKinects; i++){
        if(i==0) { rgbdProcess[i] = new RGBD_FIFO_Process(input[i], output); }
        else { rgbdProcess[i] = new RGBD_FIFO_Process(input[i], NULL); }
        
        RGBDProcessThreadTask = new std::thread(&RGBD_FIFO_Process::process, rgbdProcess[i], &calibrationFlag);
    }
    for(int i=0; i<numKinects; i++){
        RGBDProcessThreadTask[i].detach();
    }
}



void destroyAll_RGBD_FIFO_Process(){

}



void RGBD_FIFO_Process::process(bool* calibrationFlag){
    
    while(true){
        framePacket *packet = input_->get();
        framePacket* newPacket = new framePacket(packet);
        if( packet == NULL ) { break; }
        if(this->output_ != NULL){
            
            this->output_->put(newPacket);
        }
        // if calibrationFlag = false
        // just get one packet and display
        if(!*calibrationFlag){
            std::printf("One packet destroyed | FIFO length: %d | calibration = false \n", input_->cnt); fflush(stdout);
        }
        else{ // else perform calibration util success
            std::printf("One packet destroyed | FIFO length: %d | calibration = true \n", input_->cnt); fflush(stdout);
        }
        packet->destroy();
    }
    std::printf("Thread RGBD_FIFO_Process quit \n", input_->cnt); fflush(stdout);
}


void RGBD_FIFO_Process::destory(){

}


RGBD_FIFO_Process::RGBD_FIFO_Process(FIFO* input, FIFO* output){
    this->input_ = input;
    this->output_ = output;
}


RGBD_FIFO_Process::~RGBD_FIFO_Process(){

}