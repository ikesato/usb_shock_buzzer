#include <stdio.h>
#include "adxl213.h"

// ADXL213の抵抗について
// 10k[ohm] の場合
//   T2 = 10*10**3 / (125*10**6) [s]
//      = 0.08 / 10**3 [s] = 0.08[ms]
//   1/T2 = 12.5[kHz]
//   256 の分解能とすると 12.5 * 256 = 3200[kHz] = 3.2[MHz]
//
// 42k[ohm] の場合
//   T2 = 42*10**3 / (125*10**6) [s]
//      = 0.336 / 10**3 [s] = 0.336[ms]
//   1/T2 = 2.976[kHz]
//   256 の分解能とすると 2.976 * 256 = 761.9[kHz] = 0.7[MHz]
//
// 100k[ohm] の場合
//   T2 = 100*10**3 / (125*10**6) [s]
//      = 0.8 / 10**3 [s] = 0.8[ms]
//   1/T2 = 1.25[kHz]
//   256 の分解能とすると 1.25 * 256 = 320[kHz]
//
// 125k[ohm] の場合
//   T2 = 125*10**3 / (125*10**6) [s]
//      = 1 / 10**3 [s] = 1[ms]
//   1/T2 = 1[kHz]
//   256 の分解能とすると 1 * 256 = 256[kHz]
//

void adxl213_init(ADXL213 *accel)
{
    // 静止状態の値
    // TODO:とりあえず固定値だけど、動的に計算できるようにしよう
    //accel->value = 4585;
}

void adxl213_update(ADXL213 *accel, unsigned char xpin, unsigned char ypin, unsigned short now)
{
    unsigned char pins[2];
    pins[0] = xpin;
    pins[1] = ypin;

    for (int i=0; i<2; i++) {
        AccelAxis *axis = &accel->axis[i];
        if (axis->last_pin != pins[i]) {
            axis->last_pin = pins[i];
            unsigned short t;
            if (axis->last_time < now)
                t = now - axis->last_time;
            else
                t = 65536 - (axis->last_time - now);
            if (axis->last_pin == 0) {
                axis->on = t;
            } else {
                axis->off = t;
            }
            axis->last_time = now;
        }
    }
}

void adxl213_low_pass_filter(ADXL213 *accel)
{
    unsigned short v;
    for (int i=0; i<2; i++) {
        // 小数点かけ算はプログラムサイズがでかいので回避
        // この式と等価の処理を行う
        //accel->axis[i].value = (unsigned short)(accel->axis[i].value * 0.875 + accel->axis[i].on * 0.125);
        v = accel->axis[i].value;
        v = (v>>1) + (v>>2) + (v>>3);
        accel->axis[i].value = v + (accel->axis[i].on >> 3);
    }
}
