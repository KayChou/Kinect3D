#pragma once
#include "common.h"
#include "utils.h"


class Meshing {
public:
    void init(FIFO<framePacket>* input, FIFO<frameMesh>* output);
    void Loop(Context *context);

public:
    FIFO<framePacket>* input;
    FIFO<frameMesh>* output;
};