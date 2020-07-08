#ifndef ESTIMATES_LIB
#define ESTIMATES_LIB

#include <Arduino.h>
#include "BMX055.h"
#include <EEPROM.h>

#define CAL_SAMPLES 512

extern BMX055 imu;
extern uint8_t calibrate_acc;
extern float acc_calibration[3];

void get_gyr_compensated_data();
void get_acc_compensated_data();

#endif
