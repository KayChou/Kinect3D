#ifndef WIDGET_H
#define WIDGET_H

#include <iostream>
#include "KinectCapture.h"
#include "RGBD_FIFO_Process.h"
#include "PLY_Process.h"
#include "utils.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(FIFO<framePacket>** FIFO_RGBD_Acquisition, 
           FIFO<framePacket>** FIFO_RGBD_Synchronize, 
           FIFO<framePacket>** FIFO_pointCloud, 
           Context *context, 
           Ui::Widget *ui_out, 
           QWidget *parent = nullptr);
    ~Widget();

    void QtImageFIFOProcess();


signals:
    void newFrame(); // this signal is emitted if FIFO_QtImageRender not empty

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void renderNewFrame(); // display one frame in GUI

    void on_pushButton_5_clicked();

private:
    Ui::Widget *ui;
    FIFO<framePacket>** FIFO_RGBD_Acquisition;
    FIFO<framePacket>** FIFO_RGBD_Synchronize;
    FIFO<framePacket>** FIFO_pointCloud;
    FIFO<framePacket>** FIFO_QtImageRender;

private:
    Context *context;

    QImage* image;
    int indexTorender;
};
#endif // WIDGET_H
