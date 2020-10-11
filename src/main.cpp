#include "widget.h"
#include "Synchronize.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    Context* context = new Context();

    FIFO<framePacket> **RGBD_Capture = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **synchronize = new FIFO<framePacket>*[numKinects];
    FIFO<frameMesh> **pointCloud = new FIFO<frameMesh>*[numKinects];
    FIFO<framePacket> **QtImageRender = new FIFO<framePacket>*[numKinects];

    for(int i=0; i<numKinects; i++){
        RGBD_Capture[i] = new FIFO<framePacket>();
        synchronize[i] = new FIFO<framePacket>();
        pointCloud[i] = new FIFO<frameMesh>();
        QtImageRender[i] = new FIFO<framePacket>();

        RGBD_Capture[i]->init(FIFO_LEN);
        synchronize[i]->init(FIFO_LEN);
        pointCloud[i]->init(FIFO_LEN);
        QtImageRender[i]->init(FIFO_LEN);
    }

    FIFO<framePacket> **capture_output = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **synchronize_input = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **synchronize_output = new FIFO<framePacket>*[numKinects];
    FIFO<frameMesh> **opengl_render_input = new FIFO<frameMesh>*[numKinects];

    KinectsManager *kinects = new KinectsManager();
    RGBD_FIFO_Process *rgbdProcess = new RGBD_FIFO_Process[numKinects];
    openglRender *render = new openglRender();
    Ui::Widget *ui;
    QApplication a(argc, argv);
    Widget w(QtImageRender, context, ui);

    for(int i=0; i<numKinects; i++){
        capture_output[i] = RGBD_Capture[i];
        synchronize_input[i] = RGBD_Capture[i];
        synchronize_output[i] = synchronize[i];
        opengl_render_input[i] = pointCloud[i];

        rgbdProcess[i].init(synchronize[i], pointCloud[i], QtImageRender[i]);
    }
    
    kinects->init(capture_output, context);
    render->init(opengl_render_input, context);

    std::thread KinectsManager_Thread = std::thread(&KinectsManager::loop, std::ref(kinects));
    std::thread synchronize_Thread = std::thread(Synchronize, synchronize_input, synchronize_output);
    std::thread opengl_Render_Thread = std::thread(&openglRender::loop, std::ref(render));
    std::thread rgbd_Process_Thread[numKinects];
    std::thread ImageFIFOThread(&Widget::QtImageFIFOProcess, std::ref(w));

    for(int i=0; i<numKinects; i++){
        rgbd_Process_Thread[i] = std::thread(&RGBD_FIFO_Process::process, rgbdProcess[i], context);
        rgbd_Process_Thread[i].detach();
    }

    KinectsManager_Thread.detach();
    synchronize_Thread.detach();
    opengl_Render_Thread.detach();
    ImageFIFOThread.detach();

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
    delete [] capture_output;
    delete [] synchronize_input;
    delete [] synchronize_output;
    delete [] opengl_render_input;
    delete [] context;
    delete kinects;
    delete render;

    return 0;
}