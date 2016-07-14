#include "sound.h"
#include "gtime.h"
#include "util.h"

static unsigned char playno;
static unsigned short start_time=0;
static unsigned char counter = 0;

void sound_play(unsigned char pno)
{
    playno = pno;
    start_time = gtime;
}

unsigned char sound_update(void)
{
    unsigned short difft;
    counter++;
    if (playno == 1) {
        difft = diff_time(gtime, start_time);
        if (difft > TIME_QUARTER_SEC)
            return 0;
        if ((counter>>0) & 1)
            return 128;
        else
            return 0;
    } else if (playno == 2) {
        if ((counter>>0) & 1)
            return 128;
        else
            return 0;
    }
    return 0;
}
