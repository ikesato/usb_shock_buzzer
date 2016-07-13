#ifndef _util_h_
#define _util_h_

unsigned short diff_time(unsigned short now, unsigned short prev);
unsigned short calc_low_pass_filter(unsigned short current, unsigned short prev);
unsigned char calc_growing(unsigned char now);


#endif//_util_h_
