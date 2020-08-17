#include "widget.h"
#include "Synchronize.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    // define FIFO for RGBD storage
    FIFO<framePacket> **FIFO_RGBD_Acquisition = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **FIFO_RGBD_Synchronize = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **FIFO_pointCloud = new FIFO<framePacket>*[numKinects];
    for(int i=0; i<numKinects; i++){
        FIFO_RGBD_Acquisition[i] = new FIFO<framePacket>();
        FIFO_RGBD_Acquisition[i]->init(FIFO_LEN);
        FIFO_RGBD_Synchronize[i] = new FIFO<framePacket>();
        FIFO_RGBD_Synchronize[i]->init(FIFO_LEN);
        FIFO_pointCloud[i] = new FIFO<framePacket>();
        FIFO_pointCloud[i]->init(FIFO_LEN);
    }

    std::thread thread_Synchronize = std::thread(Synchronize, FIFO_RGBD_Acquisition, FIFO_RGBD_Synchronize);
    thread_Synchronize.detach();

    Ui::Widget *ui;

    // create window
    QApplication a(argc, argv);
    Widget w(FIFO_RGBD_Acquisition, FIFO_RGBD_Synchronize, FIFO_pointCloud, ui);
    w.show();
    a.exec();    

    // delete FIFO ptr
    for(int i=0; i<numKinects; i++){
        delete FIFO_RGBD_Acquisition[i];
        delete FIFO_RGBD_Synchronize[i];
        delete FIFO_pointCloud[i];
    }
    delete FIFO_RGBD_Acquisition;
    delete FIFO_RGBD_Synchronize;
    delete FIFO_pointCloud;
    return 0;
}