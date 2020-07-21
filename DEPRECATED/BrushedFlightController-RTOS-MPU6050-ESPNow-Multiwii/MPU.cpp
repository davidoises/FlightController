#include "MPU.h"

MPU::MPU(uint8_t address)
{
  dev_address = address;
}

void MPU::mpu_write(uint8_t reg, uint8_t val)
{
  uint8_t data[2] = {reg, val};
  i2cset(dev_address, data, 2, true);
}

uint8_t MPU::mpu_single_read(uint8_t reg)
{
  uint8_t data[1];
  i2cget(dev_address, reg, data, 1);
  return data[0];
}

void MPU::initialize()
{

  mpu_write(0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(50);
  mpu_write(0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  mpu_write(0x1A, GYRO_DLPF_CFG);    //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  mpu_write(0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec

  mpu_write(0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  
  uint8_t who_am_i = mpu_single_read(0x75);
  Serial.println(who_am_i, HEX);
}

void MPU::GYRO_Common()
{
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingG == 512) { // Reset g[axis] at start of calibration
        g[axis]=0;
      }
      g[axis] +=raw_gyr[axis]; // Sum up 512 readings
      gyroZero[axis]=g[axis]>>9;
    }
    calibratingG--;
    if(calibratingG == 0)
    {
      calibrationFinished = 1;
    }
  }

  for (axis = 0; axis < 3; axis++) {
    raw_gyr[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    raw_gyr[axis] = constrain(raw_gyr[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800); 
    previousGyroADC[axis] = raw_gyr[axis];
  }
}

void MPU::ACC_Common() 
{
  static int32_t a[3];
  if (calibratingA>0) {
    calibratingA--;
    for (uint8_t axis = 0; axis < 3; axis++) {
      if (calibratingA == 511) a[axis]=0;   // Reset a[axis] at start of calibration
      a[axis] +=raw_acc[axis];           // Sum up 512 readings
      accZero[axis] = a[axis]>>9; // Calculate average, only the last itteration where (calibratingA == 0) is relevant
      //global_conf.accZero[axis] = a[axis]>>9; // Calculate average, only the last itteration where (calibratingA == 0) is relevant
    }
    if (calibratingA == 0) {
      accZero[2] -= ACC_1G;   // shift Z down by ACC_1G and store values in EEPROM at end of calibration
      
      int address = 0;

      EEPROM.put(address, accZero[0]);
      address += sizeof(int16_t);
      EEPROM.put(address, accZero[1]);
      address += sizeof(int16_t);
      EEPROM.put(address, accZero[2]);
      EEPROM.commit();
    }
  }

  raw_acc[0]  -=  accZero[0];
  raw_acc[1] -=  accZero[1];
  raw_acc[2]   -=  accZero[2];
}

void MPU::Gyro_getADC ()
{
  uint8_t raw[6];

  i2cget(dev_address, 0x43, raw, 6);

  raw_gyr[0] = (raw[2] <<8 | raw[3]);
  raw_gyr[0] = raw_gyr[0]>>2;
  raw_gyr[0] = -raw_gyr[0];
  
  raw_gyr[1] = (raw[0] <<8 | raw[1]);
  raw_gyr[1] = raw_gyr[1]>>2;
  
  raw_gyr[2] = (raw[4] <<8 | raw[5]);
  raw_gyr[2] = raw_gyr[2]>>2;
  
  GYRO_Common();
}

void MPU::ACC_getADC ()
{
  uint8_t raw[6];

  i2cget(dev_address, 0x3B, raw, 6);
  
  raw_acc[0] = (raw[2] <<8 | raw[3]);
  //raw_acc[0] = raw_acc[0]>>3;
  raw_acc[0] = -raw_acc[0];
  
  raw_acc[1] = (raw[0] <<8 | raw[1]);
  //raw_acc[1] = raw_acc[1]>>3;
  //raw_acc[1] = -raw_acc[1];
  
  raw_acc[2] = (raw[4] <<8 | raw[5]);
  //raw_acc[2] = raw_acc[2]>>3;
  
  ACC_Common();
}
