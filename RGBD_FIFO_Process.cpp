#include "RGBD_FIFO_Process.h"

std::thread *RGBDProcessThreadTask;
RGBD_FIFO_Process **rgbdProcess;


void startAll_RGBD_FIFO_Process(FIFO** input, QImage* image, bool &calibrationFlag){
    RGBDProcessThreadTask = new std::thread[numKinects];
    rgbdProcess = new RGBD_FIFO_Process*[numKinects];

    for(int i=0; i<numKinects; i++){
        rgbdProcess[i] = new RGBD_FIFO_Process(input[i]);
        RGBDProcessThreadTask = new std::thread(&RGBD_FIFO_Process::process, rgbdProcess[i], &calibrationFlag, image);
    }
    for(int i=0; i<numKinects; i++){
        RGBDProcessThreadTask[i].detach();
    }
}



void destroyAll_RGBD_FIFO_Process(){

}





void RGBD_FIFO_Process::process(bool* calibrationFlag, QImage* imageOut){
    imageOut->load("/home/benjamin/Pictures/Demon Slayer5.png");
    QImage img = *imageOut;
    while(true){
        // if calibrationFlag = false
        // just get one packet and display
        if(!*calibrationFlag){
            std::printf("One packet destroyed | FIFO length: %d | calibration = false \n", input_->cnt); fflush(stdout);
            framePacket *packet = input_->get();
            if( packet == NULL ) { break; }
            packet->destroy();
        }
        else{ // else perform calibration util success
            std::printf("One packet destroyed | FIFO length: %d | calibration = true \n", input_->cnt); fflush(stdout);
            framePacket *packet = input_->get();
            if( packet == NULL ) { break; }
            packet->destroy();
        }
    }
    std::printf("Thread RGBD_FIFO_Process quit \n", input_->cnt); fflush(stdout);
}


void RGBD_FIFO_Process::destory(){

}


RGBD_FIFO_Process::RGBD_FIFO_Process(FIFO* input){
    this->input_ = input;
}


RGBD_FIFO_Process::~RGBD_FIFO_Process(){

}