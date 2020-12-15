#pragma once
#include "common.h"
#include <algorithm>

int timeStampThreshold = 300;


void Synchronize(FIFO<framePacket> **input, FIFO<framePacket> **output){
    bool stampClearFlag = false;
    int cnt[numKinects];

    uint32_t timeStampInit[numKinects];
    uint32_t timeStamp[numKinects];
    framePacket **packetList = new framePacket*[numKinects];

    uint32_t timeStampMean = 0;
    std::printf("Thread Synchronize started \n"); fflush(stdout);

    int frameCnt = 0;

    while(true){
        if(numKinects == 1) {
            framePacket *packet = input[0]->get();
            std::printf("synchronize get one frame\n"); fflush(stdout);
            output[0]->put(packet);
        }
        else{
            if(!stampClearFlag) {
                for(int i=0; i<numKinects; i++) {
                    cnt[i] = input[i]->cnt;
                }
                int max = *max_element(cnt, cnt + numKinects);
                int min = *min_element(cnt, cnt + numKinects);
                int maxIdx = max_element(cnt, cnt + numKinects) - cnt;
                //when all capture thread created, clear all FIFO and note start timeStamp
                if(min > 0){ 
                    stampClearFlag = true;
                    for(int i=0; i<numKinects; i++) {
                        while(input[i]->cnt > 0) {
                            framePacket *packet = input[i]->get();
                            timeStampInit[i] = packet->timestamp_c;
                            packet->destroy();
                        }
                    }
                }
                else if(max > 0) {
                    framePacket *packet = input[maxIdx]->get();
                    packet->destroy();
                }
                usleep(30000);
            }

            // synchronize
            else{
                for(int i=0; i<numKinects; i++) {
                    packetList[i] = input[i]->get();
                    if(packetList[i]) {
                        timeStamp[i] = packetList[i]->timestamp_c - timeStampInit[i];
                    }
                    //std::printf("%d ", timeStamp[i]); fflush(stdout);
                }
                std::printf("synchronize get one frame\n"); fflush(stdout);
                //std::printf("\n"); fflush(stdout);

                uint32_t max = *max_element(timeStamp, timeStamp + numKinects);

                for(int i=0; i<numKinects; i++) {
                    if( max - timeStamp[i] > timeStampThreshold ) {
                        packetList[i]->destroy();
                        packetList[i] = input[i]->get();
                        std::printf("FIFO %d throw away one frame\n", i); fflush(stdout);
                    }
                    output[i]->put(packetList[i]);
                }
            }
        }
        usleep(2000);
    }
    delete [] packetList;
}