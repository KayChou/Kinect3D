#include "overlap_removal.h"


void overlap_removal::init(FIFO<framePacket>** input, FIFO<framePacket>** output, Context *context)
{
    this->input = input;
    this->output = output;
    this->context = context;
    frameList = new framePacket*[numKinects];

    if(numKinects > 1) {
        for(int i=0; i<numKinects; i++) {
            this->removal[i].init(this->context);
            Thread[i] = std::thread(&twoViewRemoval::loop, &this->removal[i]);
            Thread[i].detach();
        }
    }
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

            if(cnt < 1) {
                std::printf("Begin to save one frame\n"); fflush(stdout);
                filename =  "../datas/mesh_world_left_" + std::to_string(cnt) + ".ply";
                savePlyFile(filename.c_str(), frameList[0]->vertices, Width_depth_HR * Height_depth_HR);
                filename =  "../datas/mesh_world_right_" + std::to_string(cnt) + ".ply";
                savePlyFile(filename.c_str(), frameList[1]->vertices, Width_depth_HR * Height_depth_HR);
            }

            overlap_removal_cuda(ctx_gpu, frameList);


            if(cnt < 1) {
                filename =  "../datas/mesh_camera_left_" + std::to_string(cnt) + ".ply";
                savePlyFile(filename.c_str(), frameList[0]->vertices, Width_depth_HR * Height_depth_HR);
                filename =  "../datas/mesh_camera_right_" + std::to_string(cnt++) + ".ply";
                savePlyFile(filename.c_str(), frameList[1]->vertices, Width_depth_HR * Height_depth_HR);
                std::printf("Finish save one frame\n"); fflush(stdout);
            }


#if 0
            for(int i=0; i<numKinects; i++) {
                left = frameList[i];
                right = frameList[(i+1) % numKinects];
                this->removal[i].set_io(left, right, (i+1) % numKinects);
            }

            //wait for threads to finish removal
            for(int i=0; i<numKinects; i++) {
                while(this->removal[i].readable == false) {
                    usleep(100);
                }
                this->removal[i].readable = true;
                output[i]->put(frameList[i]);
            }
#endif
        }
        for(int i=0; i<numKinects; i++) {
            output[i]->put(frameList[i]);
        }
    }
}


//=======================================================================================
// removal overlapping area between two adjacent views
//=======================================================================================
void twoViewRemoval::init(Context *context)
{
    this->is_active = true;
    this->processable = false;
    this->readable = false;
    this->context = context;
}


void twoViewRemoval::set_io(framePacket* left, framePacket* right, int idx)
{
    this->left = new framePacket(left);
    // this->left = left;
    this->right = right;
    this->idx = idx;
    this->processable = true;
    this->readable = false;
}


void twoViewRemoval::loop()
{
    Point3f tempPoint;
    int cnt_removed = 0;
    timeval t_start, t_end;
    float time_diff = 0;

    std::printf("thread two view removal started\n"); fflush(stdout);
    while(this->is_active) {
        if (!this->processable) {
            usleep(100);
            continue;
        }

        gettimeofday(&t_start, NULL);

        this->processable = false;
        cnt_removed = 0;

        for(int i=0; i< left->width_d * left->height_d; i++) {
            tempPoint.X = left->vertices[i].X;
            tempPoint.Y = left->vertices[i].Y;
            tempPoint.Z = left->vertices[i].Z;

            backforward_mapping(tempPoint, context->invR[idx], context->T[idx]);
            if(map_to_pixel(tempPoint, context->K[idx].fx, context->K[idx].fy, context->K[idx].cx, context->K[idx].cy, cnt_removed)) {
                // this->right->vertices[i].X = 2 * x_bbox_max;
                if(idx == 0) {
                    this->right->vertices[i].R = 255;
                    this->right->vertices[i].G = 0;
                    this->right->vertices[i].B = 0;
                }
                if(idx == 1) {
                    this->right->vertices[i].R = 0;
                    this->right->vertices[i].G = 255;
                    this->right->vertices[i].B = 0;
                }
                
                // this->right->vertices[i].Y = 2 * y_bbox_max;
                // this->right->vertices[i].Z = 2 * z_bbox_max;
            }
            // this->right->vertices[i].R = idx % 2 * 255;
            // this->right->vertices[i].G = 0;
            // this->right->vertices[i].B = 0;
        }

        gettimeofday(&t_end, NULL);
        time_diff = get_time_diff_ms(t_start, t_end);
#ifdef LOG
        std::printf("\t remove %d points using %f ms time\n", cnt_removed, time_diff); fflush(stdout);
#endif
        this->left->destroy();
        this->readable = true;
    }
}


// map world coordinate to camera coordinate
void twoViewRemoval::backforward_mapping(Point3f &point, std::vector<std::vector<float>> &R, std::vector<float> &T)
{
    res[0] = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
    res[1] = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
    res[2] = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

    point.X = res[0] - T[0];
    point.Y = res[1] - T[1];
    point.Z = res[2] - T[2];
}


// map camera coordinate to image
// return true if tempPoint can be mapped to right view
bool twoViewRemoval::map_to_pixel(Point3f &point, float fx, float fy, float cx, float cy, int &cnt_removed)
{
    if(point.Z == 0) {
        return false;
    }

    int x = (int)(point.X * context->x_ratio / point.Z * fx + cx - 0.5);
    int y = (int)(point.Y * context->y_ratio / point.Z * fy + cy - 0.5);

    // res[0] = this->right->vertices[y*Width_depth_HR + x].X;
    //float dist = (res[0] - point.X) + (res[1] - point.Y) + (res[2] - point.Z);

    // if(x >= 0 && x < Width_depth_HR && y >= 0 && y < Height_depth_HR && dist < 1) {
    //     cnt_removed++;
    //     return true;
    // }
    return false;
}
