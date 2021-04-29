#include "Transform2world.h"


void Transform2world::init(int idx, FIFO<framePacket>* input, FIFO<framePacket>* output, FIFO<framePacket>* output_qt) 
{
    this->idx = idx;
    this->input = input;
    this->output_qt = output_qt;
    this->output = output;
    this->R.resize(3);

	for (int i = 0; i < 3; i++) {
		this->R[i].resize(3);
		for (int j = 0; j < 3; j++) {
			this->R[i][j] = 0;
		}
	}
    this->T.resize(3);
    this->T[0] = 0;
    this->T[1] = 0;
    this->T[2] = 0;
    this->transformStruct = Transform_gpu_init();
}

//=======================================================================================
// main loop: get data from RGBD FIFO
//      if calibrate button is clicked, perform calibration
//=======================================================================================
void Transform2world::process(Context *context)
{
    // load camera posture
    if(idx == 0) {
        if(loadKRT(context)) {
            for(int i=0; i<numKinects; i++) {
                context->b_hasBeenCalibrated[i] = true;
            }
        }
    }

    timeval t_start, t_end;
    float t_delay;
    FILE *f;
    int frame_cnt = 0;
    if(idx == 0) {
        f = fopen("alignment.csv", "w");
    }

    while(true) {
        framePacket *packet = input->get();
        gettimeofday(&t_start, NULL);
        
        if(context->calibration_idx == idx + 1) {
            context->b_hasBeenCalibrated[idx] = calibrate.performCalibration(packet, T, R);
            if(context->b_hasBeenCalibrated[idx]) {
                context->T[idx][0] = T[0];
                context->T[idx][1] = T[1];
                context->T[idx][2] = T[2];
                context->R[idx][0][0] = R[0][0];
                context->R[idx][0][1] = R[0][1];
                context->R[idx][0][2] = R[0][2];
                context->R[idx][1][0] = R[1][0];
                context->R[idx][1][1] = R[1][1];
                context->R[idx][1][2] = R[1][2];
                context->R[idx][2][0] = R[2][0];
                context->R[idx][2][1] = R[2][1];
                context->R[idx][2][2] = R[2][2];
                inv_Matrix_3x3(context->R[idx], context->invR[idx]);
            } 
        }

        if(context->b_hasBeenCalibrated[idx]) { // if current camera has been calibrated
            Transform((int)packet->width_d, (int)packet->height_d, packet, context->R[idx], context->T[idx], this->transformStruct);
        }
        
        if(this->output_qt != NULL){ // FIFO for QT image render
            framePacket* newPacket = new framePacket(packet);
            this->output_qt->put(newPacket);
        }
        if(this->output != NULL) {
            framePacket* newPacket = new framePacket(packet);
            this->output->put(newPacket);
        }

#ifdef LOG
        gettimeofday(&t_end, NULL);
        t_delay = get_time_diff_ms(t_start, t_end);
        if(frame_cnt < MAX_FRAME_NUM && idx == 0) {
            fprintf(f, "%f\n", t_delay);
            frame_cnt++;
        }
        else if (frame_cnt == MAX_FRAME_NUM) {
            fclose(f);
            frame_cnt++;
        }
        std::printf("Transform2world get one frame: %f\n", t_delay);
#endif
        
        if(context->b_Refine && !context->b_refined_data_ready[idx]) { // if refine is needed but data not ready
            memcpy(context->frame_to_be_refined[idx].vertices, packet->vertices, sizeof(Point3fRGB) * packet->width_d * packet->height_d);
            memcpy(context->frame_to_be_refined[idx].data_d, packet->data_d, sizeof(float) * packet->width_d * packet->height_d);
            context->b_refined_data_ready[idx] = true;
            std::printf("Transform2world prepare one frame for ICP\n"); fflush(stdout);
        }
        packet->destroy();
        usleep(1000);
    }
    std::printf("Thread Transform2world quit \n", input->cnt); fflush(stdout);
}


void Transform2world::RotatePoint(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T)
{
	std::vector<float> res(3);
	point.X += T[0];
	point.Y += T[1];
	point.Z += T[2];

	res[0] = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res[1] = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res[2] = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	point.X = res[0];
	point.Y = res[1];
	point.Z = res[2];
}
