#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "Estimates.h"
#include "Definitions.h"

void resetAttitudePID();
void calculateAttitudePID(float* setpoints);

//extern float newAxisPID[3];
extern float attitudePID[3];

#endif
