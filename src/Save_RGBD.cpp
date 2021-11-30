#include "Save_RGBD.h"

void Save_rgbd::init(int *stop_flag, FIFO<RGBD> *input, int idx) {
    this->input = input;
    this->stop_flag = stop_flag;
    this->idx = idx;

    sprintf(dataset_path, "%s/%d", dataset_root, this->idx);
    mkdir(dataset_path, S_IRWXU | S_IRWXG | S_IRWXO);

    char path_tmp[256];
    sprintf(path_tmp, "%s/color", dataset_path);
    mkdir(path_tmp, S_IRWXU | S_IRWXG | S_IRWXO);

    sprintf(path_tmp, "%s/depth", dataset_path);
    mkdir(path_tmp, S_IRWXU | S_IRWXG | S_IRWXO);

    sprintf(path_tmp, "%s/RGBD", dataset_path);
    mkdir(path_tmp, S_IRWXU | S_IRWXG | S_IRWXO);
}


void Save_rgbd::loop() {
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

    timeval t_start, t_end;
    float t_delay;
    int frame_cnt = 0;

    while(!*stop_flag) {
        gettimeofday(&t_start, NULL);

        RGBD *ptr = input->fetch_head_ptr();

        fp_color.write(reinterpret_cast<const char*>(ptr->color), 1920 * 1080 * sizeof(float));
        fp_depth.write(reinterpret_cast<const char*>(ptr->depth), 512 * 424 * sizeof(float));
        fp_rgbd.write(reinterpret_cast<const char*>(ptr->registered), 512 * 424 * sizeof(float));

        cv::Mat img;
        cv::Mat(1080, 1920, CV_8UC4, (unsigned int*)ptr->color).copyTo(img);
        sprintf(img_filename, "%s/color/%d.png", dataset_path, frame_cnt);
        cv::imwrite(img_filename, img);

        cv::Mat(424, 512, CV_8UC4, ptr->depth).copyTo(img);
        sprintf(img_filename, "%s/depth/%d.png", dataset_path, frame_cnt);
        cv::imwrite(img_filename, img);

        cv::Mat(424, 512, CV_8UC4, ptr->registered).copyTo(img);
        sprintf(img_filename, "%s/RGBD/%d.png", dataset_path, frame_cnt);
        cv::imwrite(img_filename, img);

        input->release_head_ptr();

        frame_cnt++;
        gettimeofday(&t_end, NULL);
        t_delay = get_time_diff_ms(t_start, t_end);
        std::printf("Save one frame: %0.2f ms \n", t_delay);
    }
    std::printf("Thread rgbd saver finish\n");
    return;
}