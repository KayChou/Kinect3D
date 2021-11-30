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
            save_cam_params(this->dev->getIrCameraParams(), this->dev->getColorCameraParams());

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
        std::printf("capture one frame: %0.2f ms \n", t_delay);
    }
    if(this->cameraStarted) dev->stop();
    dev->close();
    delete pipeline;
    delete listener;
    std::printf("Thread Kinect Capture finish\n");
    return true;
}


void Kinect::save_cam_params(libfreenect2::Freenect2Device::IrCameraParams irParam, 
                             libfreenect2::Freenect2Device::ColorCameraParams colorParam) {
    char param_filename[256];
    sprintf(param_filename, "%s/%d/params.txt", dataset_root, this->idx);

    FILE *f = fopen(param_filename, "w");
    fprintf(f, "Ir CameraParams\n");
    fprintf(f, "\tfx, fy, cx, cy\n\tk1, k2, k3, p1, p2\n");
    fprintf(f, "\t%0.6f %0.6f %0.6f\n\t%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", 
                                irParam.fx, 
                                irParam.fy,
                                irParam.cx,
                                irParam.cy,
                                irParam.k1,
                                irParam.k2,
                                irParam.k3,
                                irParam.p1,
                                irParam.p2
                                );
    fprintf(f, "\nColor CameraParams: \n");
    fprintf(f, "\tIntrinsic: fx, fy, cx, cy\n");
    fprintf(f, "\t%0.6f %0.6f %0.6f %0.6f %0.6f\n", 
                                colorParam.fx, 
                                colorParam.fy,
                                colorParam.cx,
                                colorParam.cy
                                );

    fprintf(f, "\nExtrinsic(depth to color)\n");
    fprintf(f, "\tshift_d, shift_m\n");
    fprintf(f, "\tmx: xxx, yyy, xxy, yyx, xx, yy, xy, x, y, 1\n");
    fprintf(f, "\tmy: xxx, yyy, xxy, yyx, xx, yy, xy, x, y, 1\n");
    fprintf(f, "\t%0.6f %0.6f\n", 
                                colorParam.shift_d,
                                colorParam.shift_m
                                );
    fprintf(f, "\t%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", 
                                colorParam.mx_x3y0,
                                colorParam.mx_x0y3,
                                colorParam.mx_x2y1,
                                colorParam.mx_x1y2,
                                colorParam.mx_x2y0,
                                colorParam.mx_x0y2,
                                colorParam.mx_x1y1,
                                colorParam.mx_x1y0,
                                colorParam.mx_x0y1,
                                colorParam.mx_x0y0
                                );
    fprintf(f, "\t%0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", 
                                colorParam.my_x3y0,
                                colorParam.my_x0y3,
                                colorParam.my_x2y1,
                                colorParam.my_x1y2,
                                colorParam.my_x2y0,
                                colorParam.my_x0y2,
                                colorParam.my_x1y1,
                                colorParam.my_x1y0,
                                colorParam.my_x0y1,
                                colorParam.my_x0y0
                                );
    fclose(f);
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