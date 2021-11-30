#include "KinectCapture.h"

//=======================================================================================
// init one kinect
//=======================================================================================
bool Kinect::init(int idx, std::string serial, float colorExposure)
{
    this->idx = idx;
    this->serial = serial;

    this->pipeline = new libfreenect2::OpenGLPacketPipeline();
    this->dev = freenect2.openDevice(serial, pipeline); // open Kinect
    this->listener = new libfreenect2::SyncMultiFrameListener(typesDefault);

    this->dev->setColorFrameListener(listener);
    this->dev->setIrAndDepthFrameListener(listener);

    this->config.EnableBilateralFilter = true;
    this->config.EnableEdgeAwareFilter = true;
    this->config.MinDepth = 0.2f;
    this->config.MaxDepth = 4.5f;
    this->dev->setConfiguration(this->config);
    this->cameraStarted = false;
    this->colorExposure = colorExposure;

    return true;
}


//=======================================================================================
// get frame and put data to FIFO
//=======================================================================================
bool Kinect::getFrameLoop(int *stop_flag, FIFO<RGBD> *output){
    std::cout << "Thread get frame from camera started" << std::endl;
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame depth_HR(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    libfreenect2::Frame depth2rgb(1920, 1080 + 2, 4);

    timeval t_start, t_end;
    float t_delay;
    int frame_cnt = 0;

    while(!*stop_flag) {
        gettimeofday(&t_start, NULL);

        if(!this->cameraStarted) { // if not started, then start it 
            this->dev->start();
            // this->dev->setColorAutoExposure(colorExposure);
            this->dev->setColorManualExposure(30, 2);
            this->registration = new libfreenect2::Registration(this->dev->getIrCameraParams(), this->dev->getColorCameraParams());

            libfreenect2::Freenect2Device::IrCameraParams CamParam = this->dev->getIrCameraParams();
            // context->K[idx].fx = CamParam.fx;
            // context->K[idx].fy = CamParam.fy;
            // context->K[idx].cx = CamParam.cx;
            // context->K[idx].cy = CamParam.cy;
            this->cameraStarted = true;
        }

        if (!listener->waitForNewFrame(frames, 10 * 1000)) { // 10 seconds
            std::cout << "timeout!" << std::endl;
        }

        this->color = frames[libfreenect2::Frame::Color];
        this->depth = frames[libfreenect2::Frame::Depth];
        this->registration->apply(color, depth, &undistorted, &registered, true, &depth2rgb);

        RGBD* frame_pack = output->fetch_tail_ptr();
        memcpy(frame_pack->color, color->data, WIDTH_C * HEIGHT_C * 4);
        memcpy(frame_pack->depth, undistorted.data, WIDTH_D * HEIGHT_D * 4);
        memcpy(frame_pack->registered, registered.data, WIDTH_D * HEIGHT_D * 4);
        output->release_tail_ptr();

        listener->release(frames);
        frame_cnt++;

        gettimeofday(&t_end, NULL);
        t_delay = get_time_diff_ms(t_start, t_end);
        std::printf("capture one frame: %0.2f ms \n", t_delay); fflush(stdout);
    }
    if(this->cameraStarted) dev->stop();
    dev->close();
    delete pipeline;
    delete listener;
    std::printf("Thread Kinect Capture finish\n"); fflush(stdout);
    return true;
}


void KinectsManager::init(int *stop_flag, FIFO<RGBD> **output) {
    // this->DeviceSerialNumber[0] = "036669543547";
    // this->DeviceSerialNumber[1] = "021871240647";
    // this->DeviceSerialNumber[2] = "005275250347";
    // this->DeviceSerialNumber[1] = "010845342847";
    this->cameraStarted = false;
    this->stop_flag = stop_flag;
    this->output = output;
}

//=======================================================================================
// find all kinect and init: configure device serial and output FIFO
//=======================================================================================
bool KinectsManager::init_Kinect()
{
    if(CAM_NUM > freenect2.enumerateDevices()) {
        std::cerr << "The number of devices does not match the specified\n";
        return false;
    }
    
    for(int i = 0; i < CAM_NUM; i++) {
        this->DeviceSerialNumber[i] = freenect2.getDeviceSerialNumber(i);
        printf("Device serial number: %s\n", this->DeviceSerialNumber[i].c_str());
        cameras[i].init(i, DeviceSerialNumber[i], 0);
    }
    usleep(2000000);

    for(int i = 0; i < CAM_NUM; i++) {
        capture_thread[i] = std::thread(&Kinect::getFrameLoop, &this->cameras[i], stop_flag, this->output[i]);
        capture_thread[i].detach();
    }
    return true;
}


void KinectsManager::loop()
{
    while(!*this->stop_flag) {
        if(!this->cameraStarted) {
            this->init_Kinect();
            this->cameraStarted = true;
            printf("init Kinect Device\n");
        }
        usleep(10000);
    }
    usleep(2000000);
    printf("Thread Kinect manager finish\n");
    return;
}