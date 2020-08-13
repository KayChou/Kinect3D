#include "widget.h"
// #include "./build/demo_autogen/include/ui_widget.h"
// #include "./build/demo_autogen/include/moc_widget.cpp"
#include "ui_widget.h"
#include "moc_widget.cpp"

Widget::Widget(FIFO** FIFO_RGBD_Acquisition, Ui::Widget *ui_out, QWidget *parent) : QWidget(parent), ui(new Ui::Widget){
    ui->setupUi(this);
    ui_out = ui;
    this->FIFO_RGBD_Acquisition = FIFO_RGBD_Acquisition;

    bOpenFlag = false;
    bStartFlag = false;
    bCalibrationFlag = false;
    FIFO_QtImageRender = new FIFO();
    FIFO_QtImageRender->init(FIFO_LEN);

    this->image = new QImage(512, 424, QImage::Format_ARGB32);


    connect(this, SIGNAL(newFrame(int)), this, SLOT(renderNewFrame(int)));
    i = 0;
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
        startAll_RGBD_FIFO_Process(FIFO_RGBD_Acquisition, FIFO_QtImageRender, bCalibrationFlag);
        std::thread ImageFIFOThread(&Widget::QtImageFIFOProcess, this);
        ImageFIFOThread.detach();
        
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
    emit newFrame(i++);
    //ui->label->setPixmap(QPixmap::fromImage(image.scaled(512, 424)));
}

void Widget::on_pushButton_3_clicked()
{

}

void Widget::on_pushButton_4_clicked()
{

}


void Widget::renderNewFrame(int i){
    ui->label->setPixmap(QPixmap::fromImage(image->scaled(512, 424)));
    std::printf("slot processed: %d \n", i); fflush(stdout);
}


void Widget::QtImageFIFOProcess(){
    while(true){
        framePacket *packet = FIFO_QtImageRender->get();
        if( packet == NULL ) { break; }

        int h = packet->height_d;
        int w = packet->width_d;
        for(int y=0; y<h; y++){
            for(int x=0; x<w; x++){
                QRgb color = qRgb(packet->vertices[y*w + x].R, packet->vertices[y*w + x].G, packet->vertices[y*w + x].B);
                image->setPixel(x, y, color);
            }
        }

        emit newFrame(i++);
        packet->destroy();
    }
}
