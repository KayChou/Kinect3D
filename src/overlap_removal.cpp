#include "overlap_removal.h"


void overlap_removal::init(FIFO<framePacket>** input, FIFO<framePacket>** output, Context *context)
{
    this->input = input;
    this->output = output;
    this->context = context;
    frameList = new framePacket*[numKinects];
    this->ctx_gpu = create_context(context);
    this->cnt = 0;
}


void overlap_removal::loop()
{
    framePacket* left;
    framePacket* right;
    
    timeval t_start, t_end;
    float t_delay;

    float *depth_out = new float[512 * 424];

    while(true) {
        if(numKinects == 1) { // if there is only one camera, no overlapping need to be removed
            frameList[0] = input[0]->get();
#ifdef LOG
            std::printf("overlap_removal get one frame\n"); fflush(stdout);
#endif
            output[0]->put(frameList[0]);
            continue;
        }

        //get all frames from FIFO
        for(int i=0; i<numKinects; i++) {
            frameList[i] = input[i]->get();
            this->context->b_all_sensor_calibrated = true;
            if(this->context->b_hasBeenCalibrated[i] == false) {
                this->context->b_all_sensor_calibrated = false;
            }
        }
        gettimeofday(&t_start, NULL);

        if(this->context->b_all_sensor_calibrated) { // if has been calibrated, then perform overlapping removal
            updata_context(ctx_gpu, context);
            overlap_removal_cuda(ctx_gpu, frameList, depth_out);

#ifdef LOG
            gettimeofday(&t_end, NULL);
            t_delay = get_time_diff_ms(t_start, t_end);
            std::printf("overlap_removal get one frame: %f\n", t_delay); fflush(stdout);
#endif

            // cv::Mat depthmat;
            // cv::Mat(424, 512, CV_32FC1, depth_out).copyTo(depthmat);
            // cv::imwrite("depth_proj.png", depthmat);
            // cv::Mat(424, 512, CV_32FC1, frameList[0]->data_d).copyTo(depthmat);
            // cv::imwrite("depth_0.png", depthmat);
            // cv::Mat(424, 512, CV_32FC1, frameList[1]->data_d).copyTo(depthmat);
            // cv::imwrite("depth_1.png", depthmat);
        }

        for(int i=0; i<numKinects; i++) {
            output[i]->put(frameList[i]);
        }
    }
}