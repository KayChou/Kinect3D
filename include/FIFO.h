#pragma once
#include <sys/time.h>
#include <unistd.h>
#include <mutex>

template<class Data_pack>
class FIFO {
private:
    int max_size;
    Data_pack **buffer;

    int head;
    int tail;
    int cnt;

private:
    std::timed_mutex mut_head;
    std::timed_mutex mut_tail;
    std::timed_mutex mut_cnt;

public:
    void init(int len) {
        this->max_size = len;
        this->head = 0;
        this->tail = 0;
        this->cnt = 0;

        if(len > 0) {
            this->buffer = new Data_pack*[this->max_size];
            for(int i = 0; i < this->max_size; i++) {
                this->buffer[i] = new Data_pack();
            }
        }
        printf("FIFO inited\n");
    }

    bool is_empty() {
        return this->cnt == 0;
    }

    bool is_full() {
        return this->cnt == this->max_size;
    }

    void destory() {
        if(this->cnt > 0) {
            printf("WARNING: FIFO is not empty: %d!\n", this->cnt);
        }
        for(int i = 0; i < this->max_size; i++) {
            delete [] this->buffer[i];
        }
        delete [] this->buffer;
        printf("FIFO destroyed\n");
    }

    int get_cnt() {
        return this->cnt;
    }

    Data_pack* fetch_head_ptr() {
        while(true) {
            if(this->is_empty()) {
                usleep(100);
                continue;
            }

            while(this->mut_head.try_lock()) {
                return this->buffer[this->head];
            }
        }
    }

    void release_head_ptr() {
        this->head = (this->head + 1) % this->max_size;
        while(this->mut_cnt.try_lock()) {
            this->cnt--;
            this->mut_cnt.unlock();
            break;
        }
        this->mut_head.unlock();
    }

    Data_pack* fetch_tail_ptr() {
        while(true) {
            if(this->is_full()) {
                usleep(100);
                continue;
            }

            while(this->mut_tail.try_lock()) {
                return this->buffer[this->tail];
            }
        }
    }

    void release_tail_ptr() {
        this->tail = (this->tail + 1) % this->max_size;
        while(this->mut_cnt.try_lock()) {
            this->cnt++;
            this->mut_cnt.unlock();
            break;
        }
        this->mut_tail.unlock();
    }
};