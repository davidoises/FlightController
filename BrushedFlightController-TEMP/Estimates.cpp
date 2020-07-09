#include "Estimates.h"

uint16_t missing_samples = 0;

void get_gyr_compensated_data()
{
  imu.get_gyr_data();
}

void get_acc_compensated_data()
{
  static float a[3];

  imu.get_acc_data();
  
  if(calibrate_acc)
  {
    missing_samples = CAL_SAMPLES;
    calibrate_acc = 0;
  }
  if(missing_samples > 0)
  {
    missing_samples--;

    if (missing_samples == 511)
    {
      a[0]=0;
      a[1]=0;
      a[2]=0;
    }

    a[0] += imu.accelerometer.x;
    a[1] += imu.accelerometer.y;
    a[2] += imu.accelerometer.z;
    
    acc_calibration[0] = a[0]/512;
    acc_calibration[1] = a[1]/512;
    acc_calibration[2] = a[2]/512;

    if(missing_samples == 0)
    {
      acc_calibration[2] -= imu.ACC_1G;
      
      int address = 0;
      EEPROM.put(address, acc_calibration[0]);
      address += sizeof(float);
      EEPROM.put(address, acc_calibration[1]);
      address += sizeof(float);
      EEPROM.put(address, acc_calibration[2]);
      EEPROM.commit();
    }
  }

  imu.accelerometer.x -= acc_calibration[0];
  imu.accelerometer.y -= acc_calibration[1];
  imu.accelerometer.z -= acc_calibration[2];
  
}

float applyDeadband(float value, float deadband)
{
  if (abs(value) < deadband) {
    value = 0;
  } else if (value > 0) {
    value -= deadband;
  } else if (value < 0) {
    value += deadband;
  }
  return value;
}

void attitude_estimation(float dt)
{
  static float acc_x = 0;
  static float acc_y = 0;
  static float acc_z = 0;
  
  static float gyr_roll = 0;
  static float gyr_pitch = 0;

  //static uint32_t previousT;
  //uint32_t currentT = micros();
  //float dt = currentT - previousT;
  //previousT = currentT;
  
  acc_x = 0.99*acc_x + 0.01*imu.accelerometer.x*imu.accelerometer.res;
  acc_y = 0.99*acc_y + 0.01*imu.accelerometer.y*imu.accelerometer.res;
  acc_z = 0.99*acc_z + 0.01*imu.accelerometer.z*imu.accelerometer.res;

  float accMag = 100.0*(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z)/(9.81*9.81);

  float acc_roll = atan2(acc_y, sqrt(acc_x*acc_x + acc_z*acc_z));
  float acc_pitch = atan2(-1.0*acc_x, sqrt(acc_y*acc_y + acc_z*acc_z));

  gyr_roll = gyr_roll*0.95 + 0.05*imu.gyroscope.x*imu.gyroscope.res;
  gyr_pitch = gyr_pitch*0.95 + 0.05*imu.gyroscope.y*imu.gyroscope.res;

  //if (72 < accMag && accMag < 133) {
  if (80 < accMag && accMag < 120) {
    roll = 0.9*(roll + gyr_roll*dt*PI/(1000000.0*180.0)) + 0.1*acc_roll;
    pitch = 0.9*(pitch + gyr_pitch*dt*PI/(1000000.0*180.0)) + 0.1*acc_pitch;
  }
  else
  {
    roll = roll + gyr_roll*dt*PI/(1000000.0*180.0);
    pitch = pitch + gyr_pitch*dt*PI/(1000000.0*180.0); 
  }
}

void acceleration_estimation(float dt)
{
  float world_acc_z = imu.accelerometer.z*cos(pitch)*cos(roll) - imu.accelerometer.x*sin(pitch) + imu.accelerometer.y*cos(pitch)*sin(roll);
  world_acc_z -= imu.ACC_1G;

  acczSmooth = 0.95*acczSmooth + 0.05*world_acc_z;
}

#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)

uint8_t altitude_estimation()
{
  static uint32_t previousT;
  uint32_t currentT = micros();
  float dt = currentT - previousT;

  // Update with a constant period
  if (dt < UPDATE_INTERVAL)
    return 0;
  previousT = currentT;

  // Do Stuff here
}
