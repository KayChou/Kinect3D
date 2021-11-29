#include "KinectCapture.h"

int main(int argc, char *argv[])
{
    int *stop_flag = new int;
    *stop_flag = false;
    // ============================ create class instance ====================================
    KinectsManager *kinects = new KinectsManager();
    
    kinects->init(stop_flag);

    // ============================ start thread =======================================
    std::thread KinectsManager_Thread = std::thread(&KinectsManager::loop, std::ref(kinects));

    KinectsManager_Thread.detach();

    while(getchar() != 'q') {
        usleep(1000);
    }
    *stop_flag = true;
    usleep(5000000);
    delete kinects;

    return 0;
}