#ifndef SENSORFUSION_LIB
#define SENSORFUSION_LIB

#include <Arduino.h>
#include <math.h>
#include "RollingMemory.h"

#define SHIFTED_YAW 1

class SensorFusion  
{
  public:

    // Constructor/destructor
    void init(double initial_roll, double initial_pitch, double initial_yaw);
    //~SensorFusion();

    void set_yaw_offset(double offset);
    void fuse_sensors(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz);
    double get_roll();
    double get_pitch();
    double get_yaw();
    
  private:
    RollingMemory acc_roll_av;
    RollingMemory acc_pitch_av;
    RollingMemory ax_av;
    RollingMemory ay_av;
    RollingMemory az_av;
    unsigned long prev_time;
    double roll_offset;
    double pitch_offset;
    double roll;
    double pitch;
    double yaw; // Mag target is the initial value at which the yaw will calibrate
    double shifted_yaw; // shofted so that range goes from -180 to 180
    double yaw_offset;
};

#endif
