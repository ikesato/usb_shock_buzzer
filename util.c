#include "util.h"

unsigned short diff_time(unsigned short now, unsigned short prev)
{
    unsigned short t;
    if (now > prev)
        return now - prev;
    return 65536 - (prev - now);
}

unsigned short calc_low_pass_filter(unsigned short current, unsigned short prev)
{
    // 小数点かけ算はプログラムサイズがでかいので回避
    // この式と等価の処理を行う
    // (unsigned short)(prev * 0.875 + current * 0.125);
    return (prev>>1) + (prev>>2) + (prev>>3) + (current >> 3);
}
