#include "widget.h"
#include "ui_widget.h"
#include "moc_widget.cpp"


//=======================================================================================
// Button: Start
//=======================================================================================
void Widget::on_pushButton_clicked()
{
    // if device is not started, then start it
    if(!context->b_cameraStarted){
        context->b_cameraStarted = true;
        ui->pushButton->setText("Stop");
        openAllKinect(FIFO_RGBD_Acquisition); // start Kinect, each sensor save framePacket to first FIFO

        std::thread ImageFIFOThread(&Widget::QtImageFIFOProcess, this);
        ImageFIFOThread.detach();
    }
    else{ // else stop devices
        context->b_cameraStarted = false;
        ui->pushButton->setText("Start");
        destoryAllKinect();
    }
}


//=======================================================================================
// Button: Calibration
//=======================================================================================
void Widget::on_pushButton_2_clicked()
{
    context->b_Calibration = context->b_Calibration ? false : true;
    ui->pushButton_2->setText(context->b_Calibration ? "Calibrating" : "Calibrate");
}


//=======================================================================================
// Button: Refine
//=======================================================================================
void Widget::on_pushButton_3_clicked()
{
    context->b_Refine = context->b_Refine ? false : true;
}


//=======================================================================================
// Button: Save
//=======================================================================================
void Widget::on_pushButton_4_clicked()
{
    context->b_save2Local = context->b_save2Local ? false : true;
    ui->pushButton_4->setText(context->b_save2Local ? "Saving" : "Save");
}


//=======================================================================================
// Button: Switch camera
//=======================================================================================
void Widget::on_pushButton_5_clicked(){
    indexTorender = (indexTorender + 1) % numKinects;
}


//=======================================================================================
// render one new frame in ui->label
//=======================================================================================
void Widget::renderNewFrame(){
    ui->label->setPixmap(QPixmap::fromImage(image->scaled(512, 424)));
}


//=======================================================================================
// get data from FIFO
//=======================================================================================
void Widget::QtImageFIFOProcess(){
    while(true){
        for(int i=0; i<numKinects; i++){
            framePacket *packet = QtImageRender[i]->get();
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
            }
            packet->destroy();
        }
        usleep(20000);
    }
}


//=======================================================================================
// construcion
//=======================================================================================
Widget::Widget(FIFO<framePacket>** FIFO_RGBD_Acquisition, 
               FIFO<framePacket>** FIFO_RGBD_Synchronize, 
               FIFO<framePacket>** FIFO_pointCloud, 
               FIFO<framePacket>** QtImageRender, 
               Context *context, 
               Ui::Widget *ui_out, 
               QWidget *parent) : QWidget(parent), ui(new Ui::Widget)
{
    ui->setupUi(this);
    ui_out = ui;

    this->FIFO_RGBD_Acquisition = FIFO_RGBD_Acquisition;
    this->FIFO_RGBD_Synchronize = FIFO_RGBD_Synchronize;
    this->FIFO_pointCloud = FIFO_pointCloud;
    this->QtImageRender = QtImageRender;

    this->context = context;

    indexTorender = 0;

    this->image = new QImage(512, 424, QImage::Format_ARGB32);
    connect(this, SIGNAL(newFrame()), this, SLOT(renderNewFrame()));
}


//=======================================================================================
// destruct
//=======================================================================================
Widget::~Widget()
{
    delete ui;    
    delete this->image;
}