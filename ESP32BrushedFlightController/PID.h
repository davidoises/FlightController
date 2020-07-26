#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "Estimates.h"
#include "Definitions.h"

/*
 * Attitude PID reinitialization
 * This functions resets time-dependant variables like integrals
 * When PID starts operating, integral initial conditions need to be 0 to avoid incorrect error accumulation
 */
void resetAttitudePID();

/*
 * Attitude PID calculation
 * This function calculates control laws for yaw, pitch and roll ased on the provided setpoints
 * Integral terms are constrained and ignored when drone is far from hovering point, this prevents excessive error accumulation
 * Derivative term is just the derivtive of the controlled variables, rather than the derivative of the error. This allos smoother D term on the PID
 */
void calculateAttitudePID(float* setpoints);

// PID control laws get stored here
extern float attitudePID[3];

#endif
