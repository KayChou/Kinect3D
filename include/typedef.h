#pragma once

#include <stdint.h>
#include "config.h"

typedef struct RGBD {
    uint8_t color[WIDTH_C * WIDTH_C * 4];
    uint8_t registered[WIDTH_D * WIDTH_D * 4];
    float depth[WIDTH_D * WIDTH_D * 4];
    
    uint32_t time_stamp;
} RGBD;