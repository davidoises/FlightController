#include "PID.h"

//Just floated PID
float new_kp = 12.0*4.1/64.0;//5;//12;
float new_ki = 30.0*4.1/(64.0*128.0);//17;//30;
float new_kd = 23.0*3.0*4.1/32.0;//52;//23;

float new_yaw_kp = 4.1*12.0/64.0;//5;//12;
float new_yaw_ki = 4.1*45.0/8192.0;

float newErrorGyroI_YAW;
float newErrorGyroI[2];
float newDelta1[2],newDelta2[2];
float newLastGyro[2] = {0,0};

void resetAttitudePID()
{
  newErrorGyroI[ROLL] = 0;
  newErrorGyroI[PITCH] = 0;
  newErrorGyroI_YAW = 0;
}

void calculateAttitudePID(float* setpoints)
{
  
  // PITCH & ROLL
  for(uint8_t axis=0;axis<2;axis++) {
    // ERROR claculation
    float new_error = setpoints[axis] - gyroLPF[axis];
    
    //Floated PID
    // Proportional term
    float newPTerm = new_kp*new_error;
    
    // Integral term
    newErrorGyroI[axis] = constrain(newErrorGyroI[axis]+new_error,-3902,+3902);
    if (abs(gyroLPF[axis])>156) newErrorGyroI[axis] = 0;
    float newITerm = newErrorGyroI[axis]*new_ki;

    // Derivative term
    float newDelta = gyroLPF[axis] - newLastGyro[axis];
    newLastGyro[axis] = gyroLPF[axis];

    // Rolling average on derivative term
    float newDTerm = (newDelta1[axis] + newDelta2[axis] + newDelta)/3.0;
    newDelta2[axis]   = newDelta1[axis];
    newDelta1[axis]   = newDelta;

    newDTerm = newDTerm*new_kd;

    newAxisPID[axis]  = newPTerm + newITerm - newDTerm;
    
  }

  //YAW
  float new_error = setpoints[YAW] - gyroLPF[YAW];

  // Proportional term
  float newPTerm = new_error*new_yaw_kp;

  // Integral term
  newErrorGyroI_YAW += new_error*new_yaw_ki;
  if (abs(setpoints[YAW]) > 50) newErrorGyroI_YAW = 0;
  float newITerm = constrain(newErrorGyroI_YAW,-250,250);

  newAxisPID[YAW]  = newPTerm + newITerm;
}
