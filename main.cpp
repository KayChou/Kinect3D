#include "widget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    // define FIFO for RGBD storage
    FIFO **RGBD_Acquisition = new FIFO*[numKinects];
    for(int i=0; i<numKinects; i++){
        RGBD_Acquisition[i] = new FIFO();
        RGBD_Acquisition[i]->init(FIFO_LEN);
    }


    Ui::Widget *ui;

    // create window
    QApplication a(argc, argv);
    Widget w(RGBD_Acquisition, ui);
    w.show();
    a.exec();    


    // delete FIFO ptr
    for(int i=0; i<numKinects; i++){
        delete RGBD_Acquisition[i];
    }
    delete RGBD_Acquisition;
    return 0;
}