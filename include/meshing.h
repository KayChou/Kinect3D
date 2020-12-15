#pragma once
#include "common.h"


class Meshing {
public:
    void init(FIFO<framePacket>* input, FIFO<frameMesh>* output);
    void Loop(Context *context);

public:
    FIFO<framePacket>* input;
    FIFO<frameMesh>* output;
};