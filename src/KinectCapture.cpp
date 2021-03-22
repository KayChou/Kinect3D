#include "KinectCapture.h"

//=======================================================================================
// init one kinect
//=======================================================================================
bool Kinect::init(int idx, std::string serial, FIFO<framePacket>* output, float colorExposure, Context *context)
{
    this->idx = idx;
    this->serial = serial;
    this->context = context;
    this->output = output;

    this->pipeline = new libfreenect2::OpenGLPacketPipeline();
    this->dev = freenect2.openDevice(serial, pipeline); // open Kinect
    this->listener = new libfreenect2::SyncMultiFrameListener(typesDefault);

    this->dev->setColorFrameListener(listener);
    this->dev->setIrAndDepthFrameListener(listener);

    this->config.EnableBilateralFilter = true;
    this->config.EnableEdgeAwareFilter = true;
    this->config.MinDepth = 0.3f;
    this->config.MaxDepth = 4.5f;
    this->dev->setConfiguration(this->config);
    this->cameraStarted = false;
    this->colorExposure = colorExposure;
    return true;
}


//=======================================================================================
// get frame and put data to FIFO
//=======================================================================================
bool Kinect::getFrameLoop(){
    std::cout << "Thread get frame from camera started" << std::endl;
    libfreenect2::Frame undistorted_LR(512, 424, 4);
    libfreenect2::Frame depth_HR(Width_depth_HR, Height_depth_HR, 4);
    libfreenect2::Frame registered(Width_depth_HR, Height_depth_HR, 4);
    libfreenect2::Frame depth2rgb(1920, 1080 + 2, 4);

    while(true){
        if(!context->b_start_Camera && this->cameraStarted) { // if current camera is started and need to close:
            this->dev->stop();
            this->cameraStarted = false;
#if USE_RAW_DEPTH
            delete this->registration;
#else
            delete this->alignment;
#endif
        }

        if(context->b_start_Camera) { // if camera need to start

            if(!this->cameraStarted) { // if not started, then start it 
                this->dev->start();
                this->dev->setColorAutoExposure(colorExposure);
                // this->dev->setColorManualExposure(30, 3);
#if USE_RAW_DEPTH
                this->registration = new libfreenect2::Registration(this->dev->getIrCameraParams(), this->dev->getColorCameraParams());
#else
                this->alignment = new libfreenect2::Alignment(this->dev->getIrCameraParams(), this->dev->getColorCameraParams());
#endif

                libfreenect2::Freenect2Device::IrCameraParams CamParam = this->dev->getIrCameraParams();
                context->K[idx].fx = CamParam.fx;
                context->K[idx].fy = CamParam.fy;
                context->K[idx].cx = CamParam.cx;
                context->K[idx].cy = CamParam.cy;
                this->cameraStarted = true;
            }

            if (!listener->waitForNewFrame(frames, 10 * 1000)) { // 10 seconds
                std::cout << "timeout!" << std::endl; fflush(stdout);
            }

            this->color = frames[libfreenect2::Frame::Color];
            this->depth = frames[libfreenect2::Frame::Depth];
#if USE_RAW_DEPTH
            this->registration->apply(color, depth, &undistorted_LR, &registered, true, &depth2rgb);
#else
            this->alignment->gen_undistorted_LR(this->depth, &undistorted_LR);
            this->alignment->bilinearSR((float *)undistorted_LR.data, (float *)depth_HR.data, Width_depth_HR, Height_depth_HR);
            this->alignment->apply(color, &depth_HR, &registered, false, &depth2rgb);
#endif

            // cv::Mat depthmat, registeredmat;
            // cv::Mat(Height_depth_HR, Width_depth_HR, CV_32FC1, depth_HR.data).copyTo(depthmat);
            // cv::imwrite("undistored_HR.png", depthmat);
            // cv::Mat(424, 512, CV_32FC1, undistorted.data).copyTo(depthmat);
            // cv::imwrite("undistored_LR.png", depthmat);

            // cv::Mat(Height_depth_HR, Width_depth_HR, CV_8UC4, registered.data).copyTo(registeredmat);
            // cv::imwrite("registered.png", registeredmat);

            Point3fRGB *vertices = new Point3fRGB[Width_depth_HR * Height_depth_HR];
            float rgb;
            for(int i=0; i < Width_depth_HR * Height_depth_HR; i++) {
#if USE_RAW_DEPTH
                this->registration->getPointXYZRGB(&undistorted_LR, &registered, i/512, i%512, vertices[i].X, vertices[i].Y, vertices[i].Z, rgb);
#else
                this->alignment->getPointXYZRGB(&depth_HR, &registered, i/Width_depth_HR, i%Width_depth_HR, vertices[i].X, vertices[i].Y, vertices[i].Z, rgb);
#endif

                if(std::isnan(vertices[i].X) || std::isnan(vertices[i].Y) || std::isnan(vertices[i].Z)) {
                    vertices[i].X = 0;
                    vertices[i].Y = 0;
                    vertices[i].Z = 0;
                }
                const uint8_t *c = reinterpret_cast<uint8_t*>(&rgb);
                vertices[i].B = c[0];
                vertices[i].G = c[1];
                vertices[i].R = c[2];
            }

            framePacket *packet = new framePacket();
#if USE_RAW_DEPTH
            packet->init(color, depth, vertices, 1920, 1080, 512, 424);
#else
            packet->init(color, &depth_HR, vertices, 1920, 1080, Width_depth_HR, Height_depth_HR);
#endif
            this->output->put(packet);
#ifdef LOG
            std::printf("\ncapture get one frame\n");
#endif

            listener->release(frames);
        }
    }
    if(this->cameraStarted) dev->stop();
    dev->close();
    std::printf("Thread Kinect Capture finish\n"); fflush(stdout);
    delete pipeline;
    delete listener;
    return true;
}


void KinectsManager::init(FIFO<framePacket>** output, Context *context)
{
    this->output = output;
    this->context = context;
    this->cameraStarted = false;
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
        // serials[i] = freenect2.getDeviceSerialNumber(i);
        serials[i] = this->context->DeviceSerialNumber[i];
        colorExposure[i] = this->context->colorExposure[i];
        printf("Device serial number: %s\n", serials[i].c_str());
        cameras[i].init(i, serials[i], output[i], colorExposure[i], context);
    }

    for(int i=0; i<numKinects; i++) {
        capture_thread[i] = std::thread(&Kinect::getFrameLoop, &this->cameras[i]);
        capture_thread[i].detach();
    }
    return true;
}


void KinectsManager::loop()
{
    while(true) {
        if(!this->cameraStarted && context->b_start_Camera) {
            this->init_Kinect();
            this->cameraStarted = true;
            printf("init Kinect Device\n");
        }
        usleep(100000);
    }
}
