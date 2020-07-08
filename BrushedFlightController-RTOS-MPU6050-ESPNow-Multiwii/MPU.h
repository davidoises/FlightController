#ifndef MPU_H_
#define MPU_H_

#include <Arduino.h>
//#include <Wire.h>
#include "i2c_wrapper.h"
#include <EEPROM.h>

#define MPU6050_DEFAULT_ADDRESS 0x68
//#define GYRO_DLPF_CFG   0 //Default settings LPF 256Hz/8000Hz sample
//#define GYRO_DLPF_CFG   5 // 10Hz LPF
#define GYRO_DLPF_CFG   6 // 5Hz LPF

#define ACC_1G 512*8

#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0) //16.4 LSB = 1 deg/s

class MPU
{
  public:
    uint16_t calibratingG;
    uint16_t calibratingA;
    uint8_t calibrationFinished = 0;

    int16_t raw_acc[3];
    int16_t raw_gyr[3];

    // Public so EEPROM values can be stored here
    int16_t accZero[3];
  
    MPU(uint8_t address = MPU6050_DEFAULT_ADDRESS);
    
    void initialize();
    void Gyro_getADC ();
    void ACC_getADC ();
    
  private:
    uint8_t dev_address;
    
    int16_t gyroZero[3];
    
    void GYRO_Common();
    void ACC_Common();
    
    void mpu_write(uint8_t reg, uint8_t val);
    uint8_t mpu_single_read(uint8_t reg);
};

#endif
