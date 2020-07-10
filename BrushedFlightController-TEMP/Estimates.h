#ifndef ESTIMATES_LIB
#define ESTIMATES_LIB

#include <Arduino.h>
#include "BMX055.h"
#include "MS5611.h"
#include <EEPROM.h>

#define CAL_SAMPLES 512
#define BARO_CAL_SAMPLES 250

extern BMX055 imu;
extern uint8_t calibrate_acc;
extern float acc_calibration[3];
extern float roll;
extern float pitch;
extern float acczSmooth;

extern MS5611 altimeter;
extern uint8_t calibrate_alt;

void get_gyr_compensated_data();
void get_acc_compensated_data();
uint8_t get_baro_data();
float applyDeadband(float value, float deadband);
void attitude_estimation(float dt);
void acceleration_estimation(float dt);
uint8_t altitude_estimation();

#endif
