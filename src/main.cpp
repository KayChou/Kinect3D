#include "KinectCapture.h"
#include "Save_RGBD.h"
#include "Synchronize.h"

int main(int argc, char *argv[])
{
    int *stop_flag = new int;
    *stop_flag = false;

    // ============================ FIFO init ==========================================
    FIFO<RGBD> **FIFO_RGBD_Capture = new FIFO<RGBD>*[CAM_NUM];
    FIFO<RGBD> **FIFO_synchronize = new FIFO<RGBD>*[CAM_NUM];
    for(int i = 0; i < CAM_NUM; i++) {
        FIFO_RGBD_Capture[i] = new FIFO<RGBD>();
        FIFO_synchronize[i] = new FIFO<RGBD>();
        FIFO_RGBD_Capture[i]->init(30);
        FIFO_synchronize[i]->init(30);
    }

    // ============================ create class instance ==============================
    KinectsManager *kinects = new KinectsManager();
    kinects->init(stop_flag, FIFO_RGBD_Capture);

    Save_rgbd *rgbd_saver = new Save_rgbd[CAM_NUM];
    for(int i = 0; i < CAM_NUM; i++) {
        rgbd_saver[i].init(stop_flag, FIFO_synchronize[i], i);
    }

    // ============================ start thread =======================================
    std::thread KinectsManager_Thread = std::thread(&KinectsManager::loop, std::ref(kinects));
    std::thread Synchronize_Thread = std::thread(Synchronize, FIFO_RGBD_Capture, FIFO_synchronize);
    KinectsManager_Thread.detach();
    Synchronize_Thread.detach();

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

    for(int i = 0; i < CAM_NUM; i++) {
        FIFO_RGBD_Capture[i]->destory();
        FIFO_synchronize[i]->destory();
        delete [] FIFO_RGBD_Capture[i];
        delete [] FIFO_synchronize[i];
    }
    delete [] FIFO_RGBD_Capture;
    delete [] FIFO_synchronize;
    delete kinects;
    delete rgbd_saver;

    return 0;
}