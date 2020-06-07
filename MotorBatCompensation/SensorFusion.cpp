#include "SensorFusion.h"

void SensorFusion::init(double initial_roll, double initial_pitch, double initial_yaw)
{
  roll = initial_roll;
  pitch = initial_pitch;
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

  // x, y angles from gyroscope
  double gyr_roll = gx*t_delta*(PI/180.0);
  double gyr_pitch = gy*t_delta*(PI/180.0);

  // x, y angles from accelerometer
  double acc_roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2)));
  double acc_pitch = atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2)));

  // x, y complementary filter for angles
  roll = 0.9*(roll + gyr_roll) + 0.1*acc_roll;
  pitch = 0.9*(pitch + gyr_pitch) + 0.1*acc_pitch;

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
