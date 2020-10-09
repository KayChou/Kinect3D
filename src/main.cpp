#include "widget.h"
#include "Synchronize.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    Context* context = new Context();

    FIFO<framePacket> **RGBD_Capture = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **synchronize = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **pointCloud = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **QtImageRender = new FIFO<framePacket>*[numKinects];

    RGBD_FIFO_Process *rgbdProcess = new RGBD_FIFO_Process[numKinects];
    KinectsManager *kinects = new KinectsManager();

    for(int i=0; i<numKinects; i++){
        RGBD_Capture[i] = new FIFO<framePacket>();
        synchronize[i] = new FIFO<framePacket>();
        pointCloud[i] = new FIFO<framePacket>();
        QtImageRender[i] = new FIFO<framePacket>();

        RGBD_Capture[i]->init(FIFO_LEN);
        synchronize[i]->init(FIFO_LEN);
        pointCloud[i]->init(FIFO_LEN);
        QtImageRender[i]->init(FIFO_LEN);

        rgbdProcess[i].init(synchronize[i], pointCloud[i], QtImageRender[i]);
    }
    
    kinects->init(RGBD_Capture, context);

    std::thread KinectsManager_Thread = std::thread(&KinectsManager::loop, std::ref(kinects));
    std::thread synchronize_Thread = std::thread(Synchronize, RGBD_Capture, synchronize);
    std::thread opengl_Render_Thread = std::thread(&start_PLY_FIFO_Process, pointCloud, context);
    std::thread rgbd_Process_Thread[numKinects];

    for(int i=0; i<numKinects; i++){
        rgbd_Process_Thread[i] = std::thread(&RGBD_FIFO_Process::process, rgbdProcess[i], context);
        rgbd_Process_Thread[i].detach();
    }

    KinectsManager_Thread.detach();
    synchronize_Thread.detach();
    opengl_Render_Thread.detach();

    Ui::Widget *ui;
    // create window
    QApplication a(argc, argv);
    Widget w(RGBD_Capture, synchronize, pointCloud, QtImageRender, context, ui);
    w.show();
    a.exec();

    // delete FIFO ptr
    for(int i=0; i<numKinects; i++){
        delete [] RGBD_Capture[i];
        delete [] synchronize[i];
        delete [] pointCloud[i];
        delete [] QtImageRender[i];
    }
    delete [] RGBD_Capture;
    delete [] synchronize;
    delete [] pointCloud;
    delete [] QtImageRender;
    delete [] context;
    delete kinects;

    return 0;
}