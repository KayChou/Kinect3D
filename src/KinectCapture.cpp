#include "KinectCapture.h"

//=======================================================================================
// init one kinect
//=======================================================================================
bool Kinect::init(std::string serial, FIFO<framePacket>* output, Context *context){
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
    this->config.MaxDepth = 12.0f;
    this->dev->setConfiguration(this->config);
    this->cameraStarted = false;
    return true;
}


//=======================================================================================
// get frame and put data to FIFO
//=======================================================================================
bool Kinect::getFrameLoop(){
    std::cout << "Thread get frame from camera started" << std::endl;
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    while(true){
        if(!context->b_start_Camera && this->cameraStarted) { // if current camera is started and need to close:
            this->dev->stop();
            // this->dev->close();
            this->cameraStarted = false;
            delete this->registration;
        }

        if(context->b_start_Camera) { // if camera need to start

            if(!this->cameraStarted) { // if not started, then start it 
                this->dev->start();
                this->registration = new libfreenect2::Registration(this->dev->getIrCameraParams(), this->dev->getColorCameraParams());
                this->cameraStarted = true;
            }

            if (!listener->waitForNewFrame(frames, 10*1000)) { // 10 seconds
                std::cout << "timeout!" << std::endl;
            }

            this->color = frames[libfreenect2::Frame::Color];
            this->depth = frames[libfreenect2::Frame::Depth];
            this->registration->apply(color, depth, &undistorted, &registered, true, &depth2rgb);

            Point3fRGB *vertices = new Point3fRGB[depth->width * depth->height];
            float rgb;
            for(int i=0; i < depth->width * depth->height; i++){
                this->registration->getPointXYZRGB(&undistorted, &registered, i/512, i%512, vertices[i].X, vertices[i].Y, vertices[i].Z, rgb);

                if(std::isnan(vertices[i].X) || std::isnan(vertices[i].Y) || std::isnan(vertices[i].Z)){
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
            packet->init(color, depth, vertices);
            this->output->put(packet);
            std::printf("capture get one frame\n");

            listener->release(frames);
        }
        
    }
    dev->stop();
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
    if(numKinects > freenect2.enumerateDevices()){
        std::cerr << "The number of devices does not match the specified\n";
        return false;
    }
    
    for(int i=0; i<numKinects; i++) {
        serials[i] = freenect2.getDeviceSerialNumber(i);
        cameras[i].init(serials[i], output[i], context);
    }

    for(int i=0; i<numKinects; i++) {
        capture_thread[i] = std::thread(&Kinect::getFrameLoop, &this->cameras[i]);
        capture_thread[i].detach();
    }
    return true;
}


void KinectsManager::loop()
{
    while(true){
        if(!this->cameraStarted && context->b_start_Camera){
            this->init_Kinect();
            this->cameraStarted = true;
        }
        usleep(100000);
    }
}
