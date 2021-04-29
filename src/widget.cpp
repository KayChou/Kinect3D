#include "widget.h"
#include "ui_widget.h"
#include "moc_widget.cpp"

// Button: Start
void Widget::on_pushButton_clicked()
{
    // if device is not started, then start it
    if(!context->b_start_Camera){
        context->b_start_Camera = true;
        ui->pushButton->setText("Stop");
    }
    else{ // else stop devices
        context->b_start_Camera = false;
        ui->pushButton->setText("Start");
    }
}


// Button: Calibration
void Widget::on_pushButton_2_clicked()
{
    context->b_Calibration = context->b_Calibration ? false : true;
    context->calibration_idx = (context->calibration_idx + 1) % (numKinects + 1);
    ui->pushButton_2->setText(context->calibration_idx ? "Calibrating" : "Calibrate");
}


// Button: Refine
void Widget::on_pushButton_3_clicked()
{
    // context->b_Refine = context->b_Refine ? false : true;
    if(!context->b_Refine) {
        context->b_Refine = true;
    }
}


// Button: Save
void Widget::on_pushButton_4_clicked()
{
    context->b_save2Local = context->b_save2Local ? false : true;
    ui->pushButton_4->setText(context->b_save2Local ? "Saving" : "Save");
}


// Button: Switch camera
void Widget::on_pushButton_5_clicked(){
    indexTorender = (indexTorender + 1) % numKinects;
}


// render one new frame in ui->label
void Widget::renderNewFrame(){
    ui->label->setPixmap(QPixmap::fromImage(image->scaled(512, 424)));
}


// get data from FIFO
void Widget::QtImageFIFOProcess(){
    float x_ratio = (512.f - 1) / Width_depth_HR;
    float y_ratio = (424.f - 1) / Height_depth_HR;
    std::cout << "Thread Qt image process started\n";
    while(true){
        for(int i=0; i<numKinects; i++){
            framePacket *packet = QtImageRender[i]->get();
            if( packet == NULL ) { break; }
#ifdef LOG
            std::cout << "Qt image render get one frame\n";
#endif

            if(i == indexTorender){
                int h = packet->height_d;
                int w = packet->width_d;
                for(int y = 0; y < h; y++){
                    for(int x = 0; x < w; x++){
                        QRgb color = qRgb(packet->vertices[y*w + x].R, packet->vertices[y*w + x].G, packet->vertices[y*w + x].B);
                        image->setPixel((int)(x * x_ratio), (int)(y * y_ratio), color);
                    }
                }
                emit newFrame();
            }
            packet->destroy();
        }
        usleep(20000);
    }
}


// construcion
Widget::Widget(FIFO<framePacket>** QtImageRender, 
               Context *context, 
               Ui::Widget *ui_out, 
               QWidget *parent) : QWidget(parent), ui(new Ui::Widget)
{
    ui->setupUi(this);
    ui_out = ui;

    this->QtImageRender = QtImageRender;

    this->context = context;

    indexTorender = 0;

    this->image = new QImage(512, 424, QImage::Format_ARGB32);
    connect(this, SIGNAL(newFrame()), this, SLOT(renderNewFrame()));
}


// destruct
Widget::~Widget()
{
    delete ui;    
    delete this->image;
}