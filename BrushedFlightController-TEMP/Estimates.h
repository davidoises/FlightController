#ifndef ESTIMATES_LIB
#define ESTIMATES_LIB

#include <Arduino.h>
#include "BMX055.h"
#include <EEPROM.h>

#define CAL_SAMPLES 512

extern BMX055 imu;
extern uint8_t calibrate_acc;
extern float acc_calibration[3];

//extern float acc_x;
//extern float acc_y;
//extern float acc_z;

extern float roll;
extern float pitch;

extern float acczSmooth;

void get_gyr_compensated_data();
void get_acc_compensated_data();
float applyDeadband(float value, float deadband);
void attitude_estimation(float dt);
void acceleration_estimation(float dt);
uint8_t altitude_estimation();

#endif
