#include "KinectCapture.h"

float get_time_diff_ms(timeval start, timeval end)
{
    long time_ms_end =  (end.tv_sec * 1000000 + end.tv_usec);
    long time_ms_start =  (start.tv_sec * 1000000 + start.tv_usec);
    return float(time_ms_end - time_ms_start) / 1000;
}

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

    sprintf(dataset_path, "%s/%s", dataset_root, this->serial.c_str());
    mkdir(dataset_path, S_IRWXU | S_IRWXG | S_IRWXO);

    char path_tmp[256];
    sprintf(path_tmp, "%s/color", dataset_path, this->serial.c_str());
    mkdir(path_tmp, S_IRWXU | S_IRWXG | S_IRWXO);

    sprintf(path_tmp, "%s/depth", dataset_path, this->serial.c_str());
    mkdir(path_tmp, S_IRWXU | S_IRWXG | S_IRWXO);

    sprintf(path_tmp, "%s/RGBD", dataset_path, this->serial.c_str());
    mkdir(path_tmp, S_IRWXU | S_IRWXG | S_IRWXO);

    return true;
}


//=======================================================================================
// get frame and put data to FIFO
//=======================================================================================
bool Kinect::getFrameLoop(int *stop_flag){
    std::cout << "Thread get frame from camera started" << std::endl;
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame depth_HR(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    libfreenect2::Frame depth2rgb(1920, 1080 + 2, 4);

    timeval t_start, t_end;
    float t_delay;
    int frame_cnt = 0;
    std::ofstream fp_color;
    std::ofstream fp_depth;
    std::ofstream fp_rgbd;

    char img_filename[256];

    sprintf(img_filename, "%s/color.bin", dataset_path);
    fp_color.open(img_filename, std::ios::out |  std::ios::trunc | std::ios::binary);
    sprintf(img_filename, "%s/depth.bin", dataset_path);
    fp_depth.open(img_filename, std::ios::out |  std::ios::trunc | std::ios::binary);
    sprintf(img_filename, "%s/rgbd.bin", dataset_path);
    fp_rgbd.open(img_filename, std::ios::out |  std::ios::trunc | std::ios::binary);

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
            std::cout << "timeout!" << std::endl; fflush(stdout);
        }

        this->color = frames[libfreenect2::Frame::Color];
        this->depth = frames[libfreenect2::Frame::Depth];
        this->registration->apply(color, depth, &undistorted, &registered, true, &depth2rgb);

        // cv::Mat img;
        // cv::Mat(1080, 1920, CV_8UC4, (unsigned int*)color->data).copyTo(img);
        // sprintf(img_filename, "%s/color/%d.png", dataset_path, frame_cnt);
        // cv::imwrite(img_filename, img);

        // cv::Mat(424, 512, CV_8UC4, undistorted.data).copyTo(img);
        // sprintf(img_filename, "%s/depth/%d.png", dataset_path, frame_cnt);
        // cv::imwrite(img_filename, img);

        // cv::Mat(424, 512, CV_8UC4, registered.data).copyTo(img);
        // sprintf(img_filename, "%s/RGBD/%d.png", dataset_path, frame_cnt);
        // cv::imwrite(img_filename, img);

        fp_color.write(reinterpret_cast<const char*>(color->data), 1920 * 1080 * sizeof(float));
        fp_depth.write(reinterpret_cast<const char*>(undistorted.data), 512 * 424 * sizeof(float));
        fp_rgbd.write(reinterpret_cast<const char*>(registered.data), 512 * 424 * sizeof(float));

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


void KinectsManager::init(int *stop_flag) {
    // this->DeviceSerialNumber[0] = "036669543547";
    // this->DeviceSerialNumber[1] = "021871240647";
    // this->DeviceSerialNumber[2] = "005275250347";
    // this->DeviceSerialNumber[1] = "010845342847";
    this->cameraStarted = false;
    this->stop_flag = stop_flag;
}

//=======================================================================================
// find all kinect and init: configure device serial and output FIFO
//=======================================================================================
bool KinectsManager::init_Kinect()
{
    if(numKinects > freenect2.enumerateDevices()) {
        std::cerr << "The number of devices does not match the specified\n";
        return false;
    }
    
    for(int i=0; i<numKinects; i++) {
        this->DeviceSerialNumber[i] = freenect2.getDeviceSerialNumber(i);
        printf("Device serial number: %s\n", this->DeviceSerialNumber[i].c_str());
        cameras[i].init(i, DeviceSerialNumber[i], 0);
    }
    usleep(2000000);

    for(int i=0; i<numKinects; i++) {
        capture_thread[i] = std::thread(&Kinect::getFrameLoop, &this->cameras[i], stop_flag);
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