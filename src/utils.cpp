#include "utils.h"


float get_time_diff_ms(timeval start, timeval end)
{
    long time_ms_end =  (end.tv_sec * 1000000 + end.tv_usec);
    long time_ms_start =  (start.tv_sec * 1000000 + start.tv_usec);
    return float(time_ms_end - time_ms_start) / 1000;
}