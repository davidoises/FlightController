#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>
#include "MPU.h"

extern int16_t gyrSmooth[3];
extern int16_t accSmooth[3];
extern int16_t angle[2];

void getEstimatedAttitude(int16_t* acc, int16_t* gyr);

#endif
