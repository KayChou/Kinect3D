#pragma once
#include "common.h"
#include <algorithm>


void Synchronize(FIFO<framePacket> **input, FIFO<framePacket> **output){
    while(true){
        //std::printf("Camera FIFO length: %d %d\n", input[0]->cnt, input[1]->cnt); fflush(stdout);
        int cnt[numKinects];
        for(int i=0; i<numKinects; i++){
            cnt[i] = input[i]->cnt;
        }
        int max = *max_element(cnt, cnt + numKinects);
        int min = *min_element(cnt, cnt + numKinects);
        int maxIdx = max_element(cnt, cnt + numKinects) - cnt;

        if(max > min){
            framePacket *packet = input[maxIdx]->get();
            packet->destroy();
        }
        else{
            if(max > 0){
                for(int i=0; i<numKinects; i++){
                    framePacket *packet = input[i]->get();
                    output[i]->put(packet);
                }
            }
            else{
                usleep(30000);
            }
        }
        
        //std::printf("Synchronized FIFO length: %d %d\n", output[0]->cnt, output[1]->cnt); fflush(stdout);
    }
}