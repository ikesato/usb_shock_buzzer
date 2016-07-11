#include <string.h>
#include <stdlib.h>
#include "shock_detector.h"
#include "util.h"

#define STABLE_THRESHOLD_STABLE 500
#define STABLE_THRESHOLD_LITTLE 1100
#define STABLE_THRESHOLD_LARGE  1900

void shock_detector_init(ShockDetector *detector)
{
    memset(detector, 0, sizeof(ShockDetector));
}

void shock_detector_update(ShockDetector *detector, ADXL213 *accel, unsigned short now)
{
    if (detector->mode == SD_MODE_NOT_STARTED) {
        detector->last_time = now;
        return;
    }
    int i;
    unsigned short difft = diff_time(now, detector->last_time);
    unsigned short shock_value[2];
    for (i=0; i<2; i++) {
        shock_value[i] = abs(detector->axis[i].stable_value - accel->axis[i].value);
    }
    if (detector->mode == SD_MODE_STARTED) {
        for (i=0; i<2; i++) {
            detector->axis[i].stable_value = calc_low_pass_filter(accel->axis[i].value, detector->axis[i].stable_value);
            if (shock_value[i] >= STABLE_THRESHOLD_STABLE) {
                detector->last_time = now;
                difft = 0;
            }
        }
        if (difft >= 30) { // 3[sec]
            detector->mode = SD_MODE_DETECTING;
        }
    } else if (detector->mode == SD_MODE_DETECTING) {
        for (i=0; i<2; i++) {
            if (shock_value[i] >= STABLE_THRESHOLD_LITTLE && detector->shocked == SD_SHOCK_STABLE) {
                detector->shocked = SD_SHOCK_LITTLE;
                detector->last_time = now;
            } else if (shock_value[i] >= STABLE_THRESHOLD_LARGE) {
                detector->shocked = SD_SHOCK_LARGE;
                detector->last_time = now;
            } else if (shock_value[i] < STABLE_THRESHOLD_LITTLE) {
                if (difft >= 30) { // 3[sec]
                    detector->shocked = SD_SHOCK_STABLE;
                }
            }
        }
    }
}
