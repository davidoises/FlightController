#ifndef SENSORFUSION_LIB
#define SENSORFUSION_LIB

#include <Arduino.h>
#include <math.h>
#include "RollingMemory.h"

#define SHIFTED_YAW 1

class SensorFusion  
{
  public:
    float roll_offset;
    float pitch_offset;

    // Constructor/destructor
    void init(float initial_roll, float initial_pitch, float initial_yaw);
    //~SensorFusion();

    void set_yaw_offset(float offset);
    void set_acc_offsets(float rollOffset, float pitchOffset);
    void fuse_sensors(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void fuse_sensors(float ax, float ay, float az, float gx, float gy, float gz);
    float get_roll();
    float get_pitch();
    float get_yaw();
    
  private:
    RollingMemory acc_roll_av;
    RollingMemory acc_pitch_av;
    RollingMemory ax_av;
    RollingMemory ay_av;
    RollingMemory az_av;
    unsigned long prev_time;
    float roll;
    float pitch;
    float yaw; // Mag target is the initial value at which the yaw will calibrate
    float shifted_yaw; // shofted so that range goes from -180 to 180
    float yaw_offset;
};

#endif
