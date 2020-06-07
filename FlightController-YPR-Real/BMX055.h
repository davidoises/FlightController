#ifndef BMX055_LIB
#define BMX055_LIB

#include <Arduino.h>
#include "math.h"
#include "i2c_wrapper.h"

#define DEBUG 1

#define CALIBRATE_YAW 1
#define USE_MAG_CALIBRATION 1
#define USE_MAG_RAWDATA 0
#define MAG_TARGET 3.14159// 1.57079632679f = pi/2

// Accelerometer registers
#define AM_CHIPID_REG       0x00 // Chip ud should be 0xFA
#define AM_BGW_SOFT_RESET   0x14 // Reset register
#define AM_PMU_RANGE        0x0F // G range
#define AM_PMU_BW           0x10 // Bandwidth
#define AM_OFC_CTRL         0x36 // Configuracion de Fast/slow Offset Compensation
#define AM_OFC_SETTING      0x37 // Configuracion de Fast/slow Offset Compensation
#define AM_OFC_OFFSET_X     0x38 // Offset value x, 8 bits
#define AM_OFC_OFFSET_Y     0x39 // Offset value y, 8 bits
#define AM_OFC_OFFSET_Z     0x3A // Offset value z, 8 bits
#define AM_ACCD_X_LSB       0x02 // LSB Data for accelerometer in X
#define AM_ACCD_Y_LSB       0x04 // LSB Data for accelerometer in Y
#define AM_ACCD_Z_LSB       0x06 // LSB Data for accelerometer in Z

// Gyroscope registers
#define G_CHIPID_REG        0x00 // Should be 0x0F
#define G_BGW_SOFT_RESET    0x14 // Reset register
#define G_RANGE             0x0F // Range settings
#define G_BW                0x10 // Bandwidth settings
#define G_RATE_HBW          0x13 // Select (un)filtered output and shadowing enabling
#define G_A_FOC             0x32 // Fast offset compensation settings
#define G_OFC1              0x36
#define G_OFC2              0x37
#define G_OFC3              0x38
#define G_OFC4              0x39
#define G_TRIM_GP0          0x3A
#define G_RATE_X_LSB        0x02 // LSB Data for gyroscope in X
#define G_RATE_Y_LSB        0x04 // LSB Data for gyroscope in Y
#define G_RATE_Z_LSB        0x06 // LSB Data for gyroscope in Z

// Magnetometer registers
#define MAG_CHIPID_REG      0x40 // Should be 0x32
#define MAG_SOFT_RESET      0x4B // Reset register
#define MAG_MODR            0x4C
#define MAG_REPXY           0x51 
#define MAG_REPZ            0x52
#define MAG_DATAX_LSB       0x42 // LSB Data for magnetometer in X
#define MAG_DATAY_LSB       0x44 // LSB Data for magnetometer in Y
#define MAG_DATAZ_LSB       0x46 // LSB Data for magnetometer in z
#define MAG_RHALL_LSB       0x48 // LSB Data for magnetometer hall resistor

// Magnetometer extended registers
#define BMM150_DIG_X1               UINT8_C(0x5D)
#define BMM150_DIG_Y1               UINT8_C(0x5E)
#define BMM150_DIG_Z4_LSB           UINT8_C(0x62)
#define BMM150_DIG_Z4_MSB           UINT8_C(0x63)
#define BMM150_DIG_X2               UINT8_C(0x64)
#define BMM150_DIG_Y2               UINT8_C(0x65)
#define BMM150_DIG_Z2_LSB           UINT8_C(0x68)
#define BMM150_DIG_Z2_MSB           UINT8_C(0x69)
#define BMM150_DIG_Z1_LSB           UINT8_C(0x6A)
#define BMM150_DIG_Z1_MSB           UINT8_C(0x6B)
#define BMM150_DIG_XYZ1_LSB         UINT8_C(0x6C)
#define BMM150_DIG_XYZ1_MSB         UINT8_C(0x6D)
#define BMM150_DIG_Z3_LSB           UINT8_C(0x6E)
#define BMM150_DIG_Z3_MSB           UINT8_C(0x6F)
#define BMM150_DIG_XY2              UINT8_C(0x70)
#define BMM150_DIG_XY1              UINT8_C(0x71)

typedef struct {
  uint8_t chipid;
  uint16_t range;
  float res;
  float bandwidth;
  float sample_time;
  double x_offset;
  double y_offset;
  double z_offset;
  double x;
  double y;
  double z;
}SensorVars;

typedef struct {
  // trim values
  int8_t dig_x1;
  int8_t dig_y1;
  int8_t dig_x2;
  int8_t dig_y2;
  uint16_t dig_z1;
  int16_t dig_z2;
  int16_t dig_z3;
  int16_t dig_z4;
  uint8_t dig_xy1;
  int8_t dig_xy2;
  uint16_t dig_xyz1;

  // Configuration values
  uint8_t chipid;
  float res = 1.0/16.0; //uT or 1/1.6 milliGaus per LSB x*0.0625 = y*0.1
  uint8_t odr;
  String opmode;
  int repetitions_xy;
  int repetitions_z;
  

  // Magnetometer measurements
  double x_offset = -3.511194;
  double y_offset = -34.861936;
  double z_offset = -35.088470;
  double soft_iron[9] = {16.914818, -0.006747, -0.441757, -0.006747, 16.564845, -0.168402, -0.441757, -0.168402, 18.401436};
  double yaw_offset = 0;
  double x;
  double y;
  double z;
  
}MagnetometerVars;

class BMX055  
{
  public:
    // Sensor addresses
    uint8_t Addr_Accel;
    uint8_t Addr_Gyro;
    uint8_t Addr_magnet;
    uint8_t mag_cal;

    // Constructor/destructor
    BMX055(uint8_t AddrAccel, uint8_t AddrGyro, uint8_t Addrmagnet, uint8_t magcal);
    ~BMX055();

    // Accelerometer functions and struct
    SensorVars accelerometer;
    void acc_init();
    void get_acc_data();
    void get_acc_settings();
    
    // Gyroscope functions and struct
    SensorVars gyroscope;
    void gyr_init();
    void get_gyr_data();
    void get_gyr_settings();

    // Magnetometer functions and struct
    MagnetometerVars magnetometer;
    void mag_init();
    void mag_read_trim_registers();
    void get_mag_data();
    void get_mag_settings();
    void mag_yaw_calibration(uint8_t samples);
    float mag_compensate_x(int16_t mag_data_x, uint16_t data_rhall);
    float mag_compensate_y(int16_t mag_data_y, uint16_t data_rhall);
    float mag_compensate_z(int16_t mag_data_z, uint16_t data_rhall);
    
  private:
    uint8_t acc_read(uint8_t reg);
    uint8_t acc_write(uint8_t reg, uint8_t data);
    uint8_t gyr_read(uint8_t reg);
    uint8_t gyr_write(uint8_t reg, uint8_t data);
    uint8_t mag_read(uint8_t reg);
    uint8_t mag_write(uint8_t reg, uint8_t data);
    int16_t twos_comp(uint16_t val, uint8_t bits);
};

#endif
