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
