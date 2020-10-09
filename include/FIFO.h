#ifndef FIFO_H

#include "FramePacket.h"
#include <unistd.h>
#include <mutex>

template<class Data_pack> 
class FIFO{
public:
    int maximum_size;
    Data_pack ** buffer;
    int head;
    int tail;
    int cnt;
    int timeoutCnt;
public:
    std::timed_mutex mut;
public:
    void add_one(int &ptr){
        ptr = (ptr + 1) % this->maximum_size;
    }
public:
    void init(int size){
        this->maximum_size = size;
        this->head = 0;
        this->tail = 0;
        this->buffer = new Data_pack * [this->maximum_size];
        this->cnt = 0;
        this->timeoutCnt = 0;
    }


    bool is_empty(){
        return this->cnt == 0;
    }


    bool is_full(){
        return this->cnt == this->maximum_size;
    }


    void destroy(){
        if(this->cnt > 0){
            printf("WARNING: FIFO not empty!");
        }
        delete [] this->buffer;
    }

    void put(Data_pack * item){
        while(true){
            if(this->is_full()){
                usleep(1000);
                continue;
            }
            if(this->mut.try_lock()){
                if(this->is_full()){
                    this->mut.unlock();
                    usleep(500);
                    continue;
                }
                else{
                    break;
                }
            }
        }
        this->buffer[this->tail] = item;
        this->add_one(this->tail);
        this->cnt ++;
        this->mut.unlock();
    }


    Data_pack * get(){
        while(true){
            if(this->is_empty()){
                timeoutCnt++;
                // if(timeoutCnt > 5000){return NULL;}
                usleep(1000);
                continue;
            }
            if(this->mut.try_lock()){
                if(this->is_empty()){
                    this->mut.unlock();
                    usleep(500);
                    continue;
                }
                else{
                    break;
                }
            }
        }
        Data_pack * ret = this->buffer[this->head];
        this->add_one(this->head);
        this->cnt --;
        this->timeoutCnt = 0;
        this->mut.unlock();
        return ret;
    }
};
#endif
