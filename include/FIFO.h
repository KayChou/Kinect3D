#ifndef FIFO_H

#include "FramePacket.h"
#include <unistd.h>
#include <mutex>

class FIFO{
public:
    int maximum_size;
    framePacket ** buffer;
    int head;
    int tail;
    int cnt;
    int timeoutCnt;
public:
    std::timed_mutex mut;
public:
    void add_one(int &ptr);
public:
    void init(int size);
    bool is_empty();
    bool is_full();
    void destroy();
    void put(framePacket * item);
    framePacket * get();
};
#endif
