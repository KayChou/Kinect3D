#include "widget.h"
#include "Synchronize.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    Context* context = new Context();

    // ============================ FIFO init =======================================
    FIFO<framePacket> **RGBD_Capture = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **synchronize = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **transform2world = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **pointcloud_overlap_removed = new FIFO<framePacket>*[numKinects];
    FIFO<frameMesh> **mesh = new FIFO<frameMesh>*[numKinects];
    FIFO<framePacket> **QtImageRender = new FIFO<framePacket>*[numKinects];

    for(int i=0; i<numKinects; i++){
        RGBD_Capture[i] = new FIFO<framePacket>();
        synchronize[i] = new FIFO<framePacket>();
        transform2world[i] = new FIFO<framePacket>();
        pointcloud_overlap_removed[i] = new FIFO<framePacket>();
        mesh[i] = new FIFO<frameMesh>();
        QtImageRender[i] = new FIFO<framePacket>();

        RGBD_Capture[i]->init(FIFO_LEN);
        synchronize[i]->init(FIFO_LEN);
        transform2world[i]->init(FIFO_LEN);
        pointcloud_overlap_removed[i]->init(FIFO_LEN);
        mesh[i]->init(FIFO_LEN);
        QtImageRender[i]->init(FIFO_LEN);
    }

    FIFO<framePacket> **capture_output = new FIFO<framePacket>*[numKinects];

    FIFO<framePacket> **synchronize_input = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **synchronize_output = new FIFO<framePacket>*[numKinects];

    FIFO<framePacket> **transform_input = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **transform_output = new FIFO<framePacket>*[numKinects];

    FIFO<framePacket> **overlap_remove_input = new FIFO<framePacket>*[numKinects];
    FIFO<framePacket> **overlap_remove_output = new FIFO<framePacket>*[numKinects];

    FIFO<framePacket> **meshing_input = new FIFO<framePacket>*[numKinects];
    FIFO<frameMesh> **meshing_output = new FIFO<frameMesh>*[numKinects];

    FIFO<frameMesh> **opengl_render_input = new FIFO<frameMesh>*[numKinects];

    // ============================ create class instance ====================================
    KinectsManager *kinects = new KinectsManager();
    Transform2world *transform = new Transform2world[numKinects];
    overlap_removal *overlap_remove = new overlap_removal();
    Meshing *meshing = new Meshing[numKinects];
    openglRender *render = new openglRender();
    Ui::Widget *ui;
    QApplication a(argc, argv);
    Widget w(QtImageRender, context, ui);

    for(int i=0; i<numKinects; i++){
        capture_output[i] = RGBD_Capture[i];

        synchronize_input[i] = RGBD_Capture[i];
        synchronize_output[i] = synchronize[i];

        transform_input[i] = synchronize[i];
        transform_output[i] = transform2world[i];

        overlap_remove_input[i] = transform2world[i];
        overlap_remove_output[i] = pointcloud_overlap_removed[i];

        meshing_input[i] = pointcloud_overlap_removed[i];
        meshing_output[i] = mesh[i];

        opengl_render_input[i] = mesh[i];

        transform[i].init(i, transform_input[i], transform_output[i], QtImageRender[i]);
        meshing[i].init(meshing_input[i], meshing_output[i]);
    }
    
    kinects->init(capture_output, context);
    overlap_remove->init(overlap_remove_input, overlap_remove_output);
    render->init(opengl_render_input, context);

    // ============================ start thread =======================================
    std::thread KinectsManager_Thread = std::thread(&KinectsManager::loop, std::ref(kinects));
    std::thread synchronize_Thread = std::thread(Synchronize, synchronize_input, synchronize_output);
    std::thread transform_Thread[numKinects];
    std::thread overlap_remove_Thread = std::thread(&overlap_removal::loop, std::ref(overlap_remove), context);
    std::thread meshing_Thread[numKinects];
    std::thread opengl_Render_Thread = std::thread(&openglRender::loop, std::ref(render));
    std::thread ImageFIFOThread(&Widget::QtImageFIFOProcess, std::ref(w));

    for(int i=0; i<numKinects; i++){
        transform_Thread[i] = std::thread(&Transform2world::process, transform[i], context);
        transform_Thread[i].detach();
        meshing_Thread[i] = std::thread(&Meshing::Loop, meshing[i], context);
        meshing_Thread[i].detach();
    }

    KinectsManager_Thread.detach();
    synchronize_Thread.detach();
    overlap_remove_Thread.detach();
    opengl_Render_Thread.detach();
    ImageFIFOThread.detach();

    w.show();
    a.exec();

    // ============================ free memory ====================================
    for(int i=0; i<numKinects; i++){
        delete [] RGBD_Capture[i];
        delete [] synchronize[i];
        delete [] transform2world[i];
        delete [] mesh[i];
        delete [] QtImageRender[i];
    }
    delete [] RGBD_Capture;
    delete [] synchronize;
    delete [] transform2world;
    delete [] mesh;
    delete [] QtImageRender;
    
    delete [] capture_output;
    delete [] synchronize_input;
    delete [] synchronize_output;
    delete [] transform_input;
    delete [] transform_output;
    delete [] meshing_input;
    delete [] meshing_output;
    delete [] opengl_render_input;
    delete [] context;
    delete kinects;
    delete render;

    return 0;
}