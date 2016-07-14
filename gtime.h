#ifndef _gtime_h_
#define _gtime_h_

#define TIME_SEC(n) ((n)<<4)
#define TIME_HALF_SEC (1<<3)
#define TIME_QUARTER_SEC (1<<2)

extern unsigned short gtime; // 時間単位は1/16秒

#endif//_gtime_h_
