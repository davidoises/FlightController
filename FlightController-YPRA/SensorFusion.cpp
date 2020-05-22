#include "SensorFusion.h"

void SensorFusion::init(double initial_roll, double initial_pitch, double initial_yaw)
{
  roll_offset = initial_roll;
  pitch_offset = initial_pitch;

#if SHIFTED_YAW
  yaw = initial_yaw;
  shifted_yaw = 0;
#else
  yaw = initial_yaw;
  shifted_yaw = initial_yaw;
#endif
  prev_time = millis();
}

/*SensorFusion::~SensorFusion()
{}*/

void SensorFusion::set_yaw_offset(double offset)
{
  yaw_offset = offset;
}
void SensorFusion::fuse_sensors(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz)
{
  // Get time delta between updates
  unsigned long current_time = millis();
  double t_delta = ((double)current_time - (double)prev_time)/1000;
  prev_time = current_time;

  ax_av.sum -= ax_av.memory[ax_av.index];
  ax_av.memory[ax_av.index] = ax;
  ax_av.sum += ax_av.memory[ax_av.index];
  ax_av.index++;
  if(ax_av.index == ax_av.samples) ax_av.index = 0;
  ax = ax_av.sum/ax_av.samples;

  ay_av.sum -= ay_av.memory[ay_av.index];
  ay_av.memory[ay_av.index] = ay;
  ay_av.sum += ay_av.memory[ay_av.index];
  ay_av.index++;
  if(ay_av.index == ay_av.samples) ay_av.index = 0;
  ay = ay_av.sum/ay_av.samples;

  az_av.sum -= az_av.memory[az_av.index];
  az_av.memory[az_av.index] = az;
  az_av.sum += az_av.memory[az_av.index];
  az_av.index++;
  if(az_av.index == az_av.samples) az_av.index = 0;
  az = az_av.sum/az_av.samples;

  // x, y angles from gyroscope
  double gyr_roll = gx*t_delta*(PI/180.0);
  double gyr_pitch = gy*t_delta*(PI/180.0);

  // x, y angles from accelerometer
  double acc_roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) - roll_offset;
  double acc_pitch = atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2))) - pitch_offset;

  acc_roll_av.sum -= acc_roll_av.memory[acc_roll_av.index];
  acc_roll_av.memory[acc_roll_av.index] = acc_roll;
  acc_roll_av.sum += acc_roll_av.memory[acc_roll_av.index];
  acc_roll_av.index++;
  if(acc_roll_av.index == acc_roll_av.samples) acc_roll_av.index = 0;

  acc_pitch_av.sum -= acc_pitch_av.memory[acc_pitch_av.index];
  acc_pitch_av.memory[acc_pitch_av.index] = acc_pitch;
  acc_pitch_av.sum += acc_pitch_av.memory[acc_pitch_av.index];
  acc_pitch_av.index++;
  if(acc_pitch_av.index == acc_pitch_av.samples) acc_pitch_av.index = 0;
  
  // x, y complementary filter for angles
  roll = 0.99*(roll + gyr_roll) + 0.01*acc_roll_av.sum/acc_roll_av.samples;
  pitch = 0.99*(pitch + gyr_pitch) + 0.01*acc_pitch_av.sum/acc_pitch_av.samples;

  // Get roll and pitch fot compass tilt compensation
  double mag_roll = pitch;
  double mag_pitch = -roll;

  // Magnetometer tilt ompensation
  double xh = mx*cos(mag_pitch) + my*sin(mag_roll)*sin(mag_pitch) + mz*cos(mag_roll)*sin(mag_pitch);
  double yh = my*cos(mag_roll) - mz*sin(mag_roll);
  
  // Yaw angle from magnetometer
  double mag_yaw = (atan2(yh, xh) - yaw_offset)*-1.0;
  if (mag_yaw < 0) {
    mag_yaw += 2 * PI;
  }
  if (mag_yaw > 2 * PI) {
    mag_yaw -= 2 * PI;
  }

  // Yaw angle from gyroscope
  double gyr_yaw = gz*t_delta*(PI/180.0);
  yaw += gyr_yaw;
  if (yaw < 0) {
    yaw += 2 * PI;
  }
  if (yaw > 2 * PI) {
    yaw -= 2 * PI;
  }
  
  // x, y complementary filter for angles
  yaw = 0.96*yaw + 0.04*mag_yaw;
  
#if SHIFTED_YAW
  shifted_yaw = yaw - PI;
#else
  shifted_yaw = yaw;
#endif
}

double SensorFusion::get_roll()
{
  return roll;  
}

double SensorFusion::get_pitch()
{
  return pitch;
}

double SensorFusion::get_yaw()
{
  return shifted_yaw;
}
