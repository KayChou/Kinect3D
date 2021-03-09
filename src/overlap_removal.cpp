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
#ifdef LOG
        std::printf("overlap_removal get one frame\n"); fflush(stdout);
#endif

        if(this->context->b_all_sensor_calibrated) { // if has been calibrated, then perform overlapping removal
            updata_context(ctx_gpu, context);
            overlap_removal_cuda(ctx_gpu, frameList);
        }

        for(int i=0; i<numKinects; i++) {
            output[i]->put(frameList[i]);
        }
    }
}