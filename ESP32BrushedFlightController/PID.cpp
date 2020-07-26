#include "PID.h"

// Integration variables, constantly holding the sum of errors
float yawErrorIntegral;
float rollpitchErrorIntegral[2];

// Gyro derivative rolling average variables
float prevDelta[2];
float prevPrevDelta[2];
float prevGyro[2] = {0,0};

/*
 * Attitude PID reinitialization
 */
void resetAttitudePID()
{
  rollpitchErrorIntegral[ROLL] = 0;
  rollpitchErrorIntegral[PITCH] = 0;
  yawErrorIntegral = 0;
}

/*
 * Attitude PID calculation
 */
void calculateAttitudePID(float* setpoints)
{
  
  // PITCH & ROLL PID control laws
  for(uint8_t axis=0;axis<2;axis++) {
    
    // ERROR claculation
    float error = setpoints[axis] - gyroLPF[axis];
    
    // Error integral (constrained and limited to operate only when -156<Gyro<156 deg/s)
    rollpitchErrorIntegral[axis] = constrain(rollpitchErrorIntegral[axis] + error, -3902, +3902);
    if (abs(gyroLPF[axis])>156) rollpitchErrorIntegral[axis] = 0;

    // Differential calculation (discrete derivative of gyro)
    float delta = gyroLPF[axis] - prevGyro[axis];
    prevGyro[axis] = gyroLPF[axis];

    // Rolling average of 3 samples (similar to LPF) on differential term
    float deltaLPF = (delta + prevDelta[axis] + prevPrevDelta[axis])/3.0;
    prevPrevDelta[axis]   = prevDelta[axis];
    prevDelta[axis]   = delta;
 
    // PID laplace = Kp*(S-G) + Ki*(S-G)/s - Kd*(0-G)*s
    attitudePID[axis]  = rate_rollpitch_kp * error + rate_rollpitch_ki * rollpitchErrorIntegral[axis] - rate_rollpitch_kd * deltaLPF;
    
  }

  //YAW PI control law
  // ERROR claculation
  float error = setpoints[YAW] - gyroLPF[YAW];

  // Error integral (constrained and limited to operate only when -50<Gyro<50 deg/s)
  yawErrorIntegral = constrain(yawErrorIntegral + error, -250.0/rate_yaw_ki, 250.0/rate_yaw_ki);
  if (abs(setpoints[YAW]) > 50) yawErrorIntegral = 0;
  
  // PI laplace = Kp*(S-G) + Ki*(S-G)/s
  attitudePID[YAW]  = rate_yaw_kp * error + rate_yaw_ki * yawErrorIntegral;
}
