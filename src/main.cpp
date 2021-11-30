#include "KinectCapture.h"
#include "Save_RGBD.h"

int main(int argc, char *argv[])
{
    int *stop_flag = new int;
    *stop_flag = false;

    // ============================ FIFO init ==========================================
    FIFO<RGBD> **FIFO_RGBD_Capture = new FIFO<RGBD>*[CAM_NUM];
    for(int i = 0; i < CAM_NUM; i++) {
        FIFO_RGBD_Capture[i] = new FIFO<RGBD>();
        FIFO_RGBD_Capture[i]->init(30);
    }

    // ============================ create class instance ==============================
    KinectsManager *kinects = new KinectsManager();
    kinects->init(stop_flag, FIFO_RGBD_Capture);

    Save_rgbd *rgbd_saver = new Save_rgbd[CAM_NUM];
    for(int i = 0; i < CAM_NUM; i++) {
        rgbd_saver[i].init(stop_flag, FIFO_RGBD_Capture[i], i);
    }

    // ============================ start thread =======================================
    std::thread KinectsManager_Thread = std::thread(&KinectsManager::loop, std::ref(kinects));
    KinectsManager_Thread.detach();

    std::thread save_rgbd_Thread[CAM_NUM];
    for(int i = 0; i < CAM_NUM; i++) {
        save_rgbd_Thread[i] = std::thread(&Save_rgbd::loop, std::ref(rgbd_saver[i]));
        save_rgbd_Thread[i].detach();
    }

    // ============================ block the main thread ==============================
    while(getchar() != 'q') {
        usleep(1000);
    }
    *stop_flag = true;
    usleep(5000000);
    delete kinects;

    return 0;
}