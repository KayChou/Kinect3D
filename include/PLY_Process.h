#pragma once
#include "common.h"
#include "3DRealTimeRender.h"


void start_PLY_FIFO_Process(FIFO<framePacket>** input, Context *context);


void destroy_PLY_FIFO_Process();