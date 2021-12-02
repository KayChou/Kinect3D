#pragma once
#include <algorithm>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

#include "typedef.h"
#include "FIFO.h"
#include "utils.h"

int timeStampThreshold = 300;


void Synchronize(FIFO<RGBD> **input, FIFO<RGBD> **output) {
    bool stampClearFlag = false;
    int frame_cnt = 0;

    std::printf("Thread Synchronize started \n"); fflush(stdout);

    timeval t_start, t_end;
    float t_delay;

    uint32_t timeStampInit[CAM_NUM];
    uint32_t timeStamp[CAM_NUM];
    RGBD* packetList[CAM_NUM];
    int cnt[CAM_NUM];

    while(true) {
        // if CAM_NUM = 1, no synchronize needed
        if(CAM_NUM == 1) {
            RGBD *packet_in = input[0]->fetch_head_ptr();
            RGBD *packet_out = output[0]->fetch_tail_ptr();

            memcpy(packet_out, packet_in, sizeof(RGBD));

            input[0]->release_head_ptr();
            output[0]->release_tail_ptr();
        }
        else {
            if(!stampClearFlag) {
                for(int i = 0; i < CAM_NUM; i++) {
                    cnt[i] = input[i]->get_cnt();
                }
                int max = *std::max_element(cnt, cnt + CAM_NUM);
                int min = *std::min_element(cnt, cnt + CAM_NUM);
                int maxIdx = std::max_element(cnt, cnt + CAM_NUM) - cnt;
                //when all capture thread created, clear all FIFO and record start timeStamp
                if(min > 0) {
                    stampClearFlag = true;
                    for(int i = 0; i < CAM_NUM; i++) {
                        while(input[i]->get_cnt() > 0) {
                            RGBD *packet = input[i]->fetch_head_ptr();
                            timeStampInit[i] = packet->time_stamp;
                            input[i]->release_head_ptr();
                        }
                    }
                }
                else if(max > 0) {
                    RGBD *packet = input[maxIdx]->fetch_head_ptr();
                    input[maxIdx]->release_head_ptr();
                }
                usleep(30000);
            }

            // synchronize
            else {
                gettimeofday(&t_start, NULL);

                for(int i = 0; i < CAM_NUM; i++) {
                    packetList[i] = input[i]->fetch_head_ptr();
                    timeStamp[i] = packetList[i]->time_stamp - timeStampInit[i];
                }

                uint32_t max = *std::max_element(timeStamp, timeStamp + CAM_NUM);

                for(int i = 0; i < CAM_NUM; i++) {
                    if(max - timeStamp[i] > timeStampThreshold ) {
                        input[i]->release_head_ptr();
                        packetList[i] = input[i]->fetch_head_ptr();
                    }
                    RGBD* packet = output[i]->fetch_tail_ptr();
                    memcpy(packet, packetList[i], sizeof(RGBD));
                    output[i]->release_tail_ptr();
                    input[i]->release_head_ptr();
                }
                gettimeofday(&t_end, NULL);
                t_delay = get_time_diff_ms(t_start, t_end);
                std::printf("Synchronize one frame: %0.2f ms \n", t_delay);
            }
        }
        usleep(2000);
    }
}