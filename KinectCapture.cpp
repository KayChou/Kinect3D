#include "KinectCapture.h"

int* types_;
std::string* serials_;
std::thread* kinectThreadTask;
oneKinect** devices_;

//=======================================================================================
// Open all kinect
//=======================================================================================
bool openAllKinect(int numOfKinects, FIFO<framePacket>** output){
    libfreenect2::Freenect2 freenect2_;
    if(numOfKinects > freenect2_.enumerateDevices()){
        std::cerr << "The number of devices does not match the specified\n";
        return false;
    }

    types_ = new int[numOfKinects];
    serials_ = new std::string[numOfKinects];
    kinectThreadTask = new std::thread[numOfKinects];
    devices_ = new oneKinect*[numOfKinects];
    
    for(int i=0; i<numOfKinects; i++){
        serials_[i] = freenect2_.getDeviceSerialNumber(i);
        devices_[i] = new oneKinect(serials_[i], output[i]);
        devices_[i]->setStartFlag(true);
    }

    for(int i=0; i<numOfKinects; i++){
        kinectThreadTask[i] = std::thread(&oneKinect::getFrameLoop, std::ref(devices_[i]));
        kinectThreadTask[i].detach();
    }
    return true;
}


//=======================================================================================
// close all kinects and free space 
//=======================================================================================
void destoryAllKinect(int numOfKinects){
    delete types_;
    
    for(int i=0; i<numOfKinects; i++){
        devices_[i]->setStartFlag(false);
        while(devices_[i]->getFinishFlag()){
            delete devices_[i];
        }
    }
    delete devices_;
}


//=======================================================================================
// construct one kinect
//=======================================================================================
oneKinect::oneKinect(std::string serial, FIFO<framePacket>* output, int types){
    this->output_ = output;
    this->startFlag = false;
    this->finishFlag = false;
    init(serial, types);
}


oneKinect::~oneKinect(){
    //delete pipeline_;
}


void oneKinect::setStartFlag(bool flag){
    this->startFlag = flag;
}


bool oneKinect::getFinishFlag(){
    return this->finishFlag;
}


//=======================================================================================
// init one kinect
//=======================================================================================
bool oneKinect::init(std::string serial, int types){
    serial_ = serial;
    types_ = types;

    pipeline_ = new libfreenect2::OpenGLPacketPipeline();
    dev_ = freenect2_.openDevice(serial_, pipeline_);
    listener_ = new libfreenect2::SyncMultiFrameListener(types_);

    dev_->setColorFrameListener(listener_);
    dev_->setIrAndDepthFrameListener(listener_);

    if(!dev_->start()){ return false; }

    config_.EnableBilateralFilter = true;
    config_.EnableEdgeAwareFilter = true;
    config_.MinDepth = 0.3f;
    config_.MaxDepth = 12.0f;
    dev_->setConfiguration(config_);

    registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());
    return true;
}


//=======================================================================================
// get frame and put data to FIFO
//=======================================================================================
bool oneKinect::getFrameLoop(){
    std::cout << "Thread get frame from camera started" << std::endl;
    int framecount = 0;
    clock_t start, end;
    // get image from kinect v2
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    while(startFlag){
        //start = clock();
        if (!listener_->waitForNewFrame(frames_, 10*1000)) { // 10 seconds
            std::cout << "timeout!" << std::endl;
            return -1;
        }

        color_ = frames_[libfreenect2::Frame::Color];
        depth_ = frames_[libfreenect2::Frame::Depth];
        registration_->apply(color_, depth_, &undistorted, &registered, true, &depth2rgb);

        Point3fRGB *vertices = new Point3fRGB[depth_->width * depth_->height];
        float rgb;
        int nanCnt = 0;
        for(int i=0; i < depth_->width * depth_->height; i++){
            registration_->getPointXYZRGB(&undistorted, &registered, i/512, i%512, vertices[i].X, vertices[i].Y, vertices[i].Z, rgb);

            if(std::isnan(vertices[i].X) | std::isnan(vertices[i].Y) | std::isnan(vertices[i].Z)){
                vertices[i].X = 0;
                vertices[i].Y = 0;
                vertices[i].Z = 0;
                nanCnt ++;
            }

            //if(vertices[i].Z > 0 && vertices[i].Z < 4.5){
                const uint8_t *c = reinterpret_cast<uint8_t*>(&rgb);
                vertices[i].B = c[0];
                vertices[i].G = c[1];
                vertices[i].R = c[2];
            //}
        }

        //std:printf("frame: %d has %d nan pixels\n", framecount, nanCnt); fflush(stdout);
        framePacket *packet = new framePacket();
        packet->init(color_, &undistorted, vertices);
        this->output_->put(packet);
        //std::printf("One packet is pushed | FIFO length: %d \n", output_->cnt); fflush(stdout);
        //packet->destroy();

        listener_->release(frames_);
        //end = clock();
        //std::printf("Frame count %-4d | FIFO length: %-4d | fps: %f\n", framecount, output_->cnt, CLOCKS_PER_SEC/(double)(end - start));
        framecount++;
    }

    dev_->stop();
    dev_->close();
    std::printf("Thread Kinect Capture finish\n"); fflush(stdout);
    this->finishFlag = true;
    delete pipeline_;
    delete listener_;
    return true;
}
