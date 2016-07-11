#ifndef _shock_detector_h_
#define _shock_detector_h_

#include "adxl213.h"

typedef struct ShockDetectorAxis_t {
    unsigned short stable_value;
} ShockDetectorAxis;

typedef struct ShockDetector_t {
    unsigned char mode;
    unsigned char shocked; // 0:stable 1:little 2:large
    unsigned short last_time;
    ShockDetectorAxis axis[2];
} ShockDetector;

#define SD_MODE_NOT_STARTED     0
#define SD_MODE_STARTED         1
#define SD_MODE_DETECTING       2

#define SD_SHOCK_STABLE         0
#define SD_SHOCK_LITTLE         1
#define SD_SHOCK_LARGE          2


void shock_detector_init(ShockDetector *detector);
void shock_detector_update(ShockDetector *detector, ADXL213 *accel, unsigned short now);

#endif//_shock_detector_h_
