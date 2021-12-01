#include "utils.h"
#include "registeration.h"
#include "opencv_head.h"

int main()
{
    ColorCameraParams color_param[CAM_NUM];
    IrCameraParams ir_param[CAM_NUM];

    // load param for each camera
    char filename[100];
    sprintf(filename, "%s/0-params.txt", dataset_root);
    load_params_ir_color(filename, color_param[0], ir_param[0]);
    // sprintf(filename, "%s/1-params.txt", dataset_root);
    // load_params_ir_color(filename, color_param[1], ir_param[1]);

    // load a pair of rgb and depth image
    sprintf(filename, "%s/frames/frame_0/color/0.png", dataset_root);
    cv::Mat color_img = cv::imread(filename);

    sprintf(filename, "%s/frames/frame_0/depth/0.png", dataset_root);
    cv::Mat depth_img = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);

    uint8_t *color_data = (uint8_t*)color_img.data; // 1920 * 1080 * 3, BGR, without A
    float* depth_data = (float*)depth_img.data;

    // register rgb image and depth to rgbd
    uint8_t* rgbd = new uint8_t[512 * 424 * 3];
    registeration *reg = new registeration();
    reg->init(color_param[0], ir_param[0]);
    reg->register_rgbd(color_data, depth_data, rgbd);

    cv::Mat img;
    cv::Mat(424, 512, CV_8UC3, rgbd).copyTo(img);
    cv::imwrite("../rgbd.png", img);
    
    delete reg;
    delete [] rgbd;
    return 0;
}