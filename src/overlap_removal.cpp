#include "overlap_removal.h"


void overlap_removal::init(FIFO<framePacket>** input, FIFO<framePacket>** output)
{
    this->input = input;
    this->output = output;
}


void overlap_removal::loop(Context *context)
{
    while(true) {
        if(numKinects == 1) { // if there is only one camera, no overlapping need to be removed
            framePacket *packet = input[0]->get();
            std::printf("overlap_removal get one frame\n"); fflush(stdout);
            output[0]->put(packet);
        }
    }
}


void twoViewRemoval::init()
{
    this->is_active = true;
    this->processable = false;
    this->readable = false;
}


void twoViewRemoval::set_io(framePacket* left, framePacket* right)
{
    this->left = left;
    this->right = right;
    this->processable = true;
}


void twoViewRemoval::loop()
{
    while(this->is_active) {
        if (!this->processable) {
            usleep(100);
            continue;
        }
    }
}