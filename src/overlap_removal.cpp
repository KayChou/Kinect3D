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
    timeval t_start_filter, t_end_filter;
    float t_delay;

    float *depth_out = new float[512 * 424];
    int frame_cnt = 0;
    FILE *f = fopen("overlap_removal.csv", "w");

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
            if(frame_cnt < MAX_FRAME_NUM) {
                fprintf(f, "%f\n", t_delay);
                frame_cnt++;
            }
            else if(frame_cnt == MAX_FRAME_NUM) {
                frame_cnt++;
                fclose(f);
            }
            std::printf("overlap_removal get one frame: %f\n", t_delay); fflush(stdout);
#endif
        }

        for(int i=0; i<numKinects; i++) {
            output[i]->put(frameList[i]);
        }
        
    }
}