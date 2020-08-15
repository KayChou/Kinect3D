#include "widget.h"
// #include "./build/demo_autogen/include/ui_widget.h"
// #include "./build/demo_autogen/include/moc_widget.cpp"
#include "ui_widget.h"
#include "moc_widget.cpp"


void Widget::on_pushButton_clicked()
{
    // if device is not started, then start it
    if(!bStartFlag){
        bStartFlag = true;
        ui->pushButton->setText("Stop");
        openAllKinect(numKinects, FIFO_RGBD_Acquisition); // start Kinect, each sensor save framePacket to first FIFO
        
        startAll_RGBD_FIFO_Process(FIFO_RGBD_Acquisition, FIFO_QtImageRender, bCalibrationFlag); // process first FIFO process
        std::thread ImageFIFOThread(&Widget::QtImageFIFOProcess, this);
        ImageFIFOThread.detach();
    }
    else{ // else stop devices
        bStartFlag = false;
        ui->pushButton->setText("Start");
        destoryAllKinect(numKinects);
    }
}


void Widget::on_pushButton_2_clicked()
{
    bCalibrationFlag = bCalibrationFlag ? false : true;
}


void Widget::on_pushButton_3_clicked()
{
    bRefineFlag = bRefineFlag ? false : true;
}


void Widget::on_pushButton_4_clicked()
{
    
}


void Widget::on_pushButton_5_clicked(){
    indexTorender = (indexTorender + 1) % numKinects;
}


void Widget::renderNewFrame(){
    ui->label->setPixmap(QPixmap::fromImage(image->scaled(512, 424)));
}


void Widget::QtImageFIFOProcess(){
    while(true){
        for(int i=0; i<numKinects; i++){
            framePacket *packet = FIFO_QtImageRender[i]->get();
            if( packet == NULL ) { break; }

            if(i == indexTorender){
                int h = packet->height_d;
                int w = packet->width_d;
                for(int y=0; y<h; y++){
                    for(int x=0; x<w; x++){
                        QRgb color = qRgb(packet->vertices[y*w + x].R, packet->vertices[y*w + x].G, packet->vertices[y*w + x].B);
                        image->setPixel(x, y, color);
                    }
                }

                emit newFrame();
                packet->destroy();
            }
        }
    }
}


Widget::Widget(FIFO<framePacket>** FIFO_RGBD_Acquisition, Ui::Widget *ui_out, QWidget *parent) : QWidget(parent), ui(new Ui::Widget){
    ui->setupUi(this);
    ui_out = ui;
    this->FIFO_RGBD_Acquisition = FIFO_RGBD_Acquisition;

    bOpenFlag = false;
    bStartFlag = false;
    bCalibrationFlag = false;
    bRefineFlag = false;
    indexTorender = 0;

    this->image = new QImage(512, 424, QImage::Format_ARGB32);
    connect(this, SIGNAL(newFrame()), this, SLOT(renderNewFrame()));

    FIFO_QtImageRender = new FIFO<framePacket>*[numKinects];
    for(int i=0; i<numKinects; i++){
        FIFO_QtImageRender[i] = new FIFO<framePacket>();
        FIFO_QtImageRender[i]->init(FIFO_LEN);
    }
}


Widget::~Widget()
{
    delete ui;

    for(int i=0; i<numKinects; i++){
        delete FIFO_QtImageRender[i];
    }
    delete FIFO_QtImageRender;
}