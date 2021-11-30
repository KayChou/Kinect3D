#pragma once

#include "common.h"
#include "FIFO.h"
#include "typedef.h"
#include "utils.h"

class Save_rgbd
{
public:
    FIFO<RGBD> *input;
    int *stop_flag;
    int idx;

    char dataset_path[256];

public:
    void init(int *stop_flag, FIFO<RGBD> *input, int idx);
    void loop();
};