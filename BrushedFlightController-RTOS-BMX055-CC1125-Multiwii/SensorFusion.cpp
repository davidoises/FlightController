#include "SensorFusion.h"

void SensorFusion::init(float initial_roll, float initial_pitch, float initial_yaw)
{
  //roll_offset = initial_roll;
  //pitch_offset = initial_pitch;
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

void SensorFusion::set_yaw_offset(float offset)
{
  yaw_offset = offset;
}
void SensorFusion::fuse_sensors(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  // Get time delta between updates
  unsigned long current_time = millis();
  float t_delta = ((float)current_time - (float)prev_time)/1000;
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

  /*Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.println(az);*/

  // x, y angles from gyroscope
  float gyr_roll = gx*t_delta*(PI/180.0);
  float gyr_pitch = gy*t_delta*(PI/180.0);

  // x, y angles from accelerometer
  //float acc_roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) - roll_offset;
  //float acc_pitch = atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2))) - pitch_offset;

  float acc_roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) - (-0.008483667);
  float acc_pitch = atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2))) - (0.019847667);

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
  float mag_roll = pitch;
  float mag_pitch = -roll;

  // Magnetometer tilt ompensation
  float xh = mx*cos(mag_pitch) + my*sin(mag_roll)*sin(mag_pitch) + mz*cos(mag_roll)*sin(mag_pitch);
  float yh = my*cos(mag_roll) - mz*sin(mag_roll);
  
  // Yaw angle from magnetometer
  float mag_yaw = (atan2(yh, xh) - yaw_offset)*-1.0;
  if (mag_yaw < 0) {
    mag_yaw += 2 * PI;
  }
  if (mag_yaw > 2 * PI) {
    mag_yaw -= 2 * PI;
  }

  // Yaw angle from gyroscope
  float gyr_yaw = gz*t_delta*(PI/180.0);
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
void SensorFusion::fuse_sensors(float ax, float ay, float az, float gx, float gy, float gz)
{
  // Get time delta between updates
  unsigned long current_time = millis();
  float t_delta = ((float)current_time - (float)prev_time)/1000;
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
  float gyr_roll = gx*t_delta*(PI/180.0);
  float gyr_pitch = gy*t_delta*(PI/180.0);

  float acc_roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) - (-0.09205);
  float acc_pitch = atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2))) - (-0.05691);

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
}

float SensorFusion::get_roll()
{
  return roll;  
}

float SensorFusion::get_pitch()
{
  return pitch;
}

float SensorFusion::get_yaw()
{
  return shifted_yaw;
}
