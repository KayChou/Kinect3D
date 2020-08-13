#include "widget.h"
// #include "./build/demo_autogen/include/ui_widget.h"
// #include "./build/demo_autogen/include/moc_widget.cpp"
#include "ui_widget.h"
#include "moc_widget.cpp"

Widget::Widget(FIFO** FIFO_RGBD_Acquisition, QWidget *parent) : QWidget(parent), ui(new Ui::Widget){
    ui->setupUi(this);
    this->FIFO_RGBD_Acquisition = FIFO_RGBD_Acquisition;

    bOpenFlag = false;
    bStartFlag = false;
    bCalibrationFlag = false;
}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton_clicked()
{
    // if device is not started, then start it
    if(!bStartFlag){
        bStartFlag = true;
        ui->pushButton->setText("Stop");
        openAllKinect(numKinects, FIFO_RGBD_Acquisition);
        startAll_RGBD_FIFO_Process(FIFO_RGBD_Acquisition, &image, bCalibrationFlag);
        ui->label->setPixmap(QPixmap::fromImage(image.scaled(512, 424)));
    }
    else{ // else stop devices
        bStartFlag = false;
        ui->pushButton->setText("Start");
        destoryAllKinect(numKinects);
    }

    // if(!bStartFlag){
    //     ui->pushButton->setText("Stop");
    //     QImage image;
    //     image.load("/home/benjamin/Pictures/Demon Slayer5.png");
    //     ui->label->setPixmap(QPixmap::fromImage(image.scaled(512, 424)));
    //     bStartFlag = true;
    // }
    // else{
    //     ui->pushButton->setText("Start");
    //     ui->label->clear();
    //     bStartFlag = false;
    // }
    
}


void Widget::on_pushButton_2_clicked()
{
    bCalibrationFlag = bCalibrationFlag ? false : true;
}

void Widget::on_pushButton_3_clicked()
{

}

void Widget::on_pushButton_4_clicked()
{

}
