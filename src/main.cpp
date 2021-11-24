#include "KinectCapture.h"

int main(int argc, char *argv[])
{
    // ============================ create class instance ====================================
    KinectsManager *kinects = new KinectsManager();
    
    kinects->init();

    // ============================ start thread =======================================
    std::thread KinectsManager_Thread = std::thread(&KinectsManager::loop, std::ref(kinects));

    KinectsManager_Thread.detach();

    while(getchar() != 27) {
        usleep(1000);
    }
    delete kinects;

    return 0;
}