#ifndef _adxl213_h_
#define _adxl213_h_

typedef struct AccelAxis_t {
    unsigned short value;
    unsigned short on;
    unsigned short off;
    unsigned short last_time;
    unsigned char last_pin;
    unsigned char pos;
} AccelAxis;

typedef struct ADXL213_t {
    AccelAxis axis[2]; // x and y axis
} ADXL213;

void adxl213_init(ADXL213 *accel);
void adxl213_update(ADXL213 *accel, unsigned char xpin, unsigned char ypin, unsigned short now);
void adxl213_low_pass_filter(ADXL213 *accel);

#endif//_adxl213_h_
