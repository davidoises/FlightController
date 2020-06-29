#include "BMX055.h"

BMX055::BMX055(uint8_t AddrAccel, uint8_t AddrGyro, uint8_t AddrMagnet, uint8_t magcal)
{
  Addr_Accel = AddrAccel;
  Addr_Gyro = AddrGyro;
  Addr_magnet = AddrMagnet;
  mag_cal = magcal;
}

BMX055::~BMX055()
{}

void BMX055::acc_init()
{
  // Reset accelerometer and wait 1.8ms
  acc_write(AM_BGW_SOFT_RESET, 0xB6);
  delay(4);

  // Set G range to 0x03 = +/- 2g, 0x08 = +/- 8g, needs to be in 2g for fast offset compnesation
  acc_write(AM_PMU_RANGE, 0x03);

  // St bandwidth to 0x08 = 7.81Hz, 0x0B = 62.5
  acc_write(AM_PMU_BW, 0x0B);

  // Completely disable FOC and reset offsets
  //acc_write(AM_OFC_CTRL, 0x00);

  /*
  // Set offset targets to z=1g, y=0g, x0g
  acc_write(AM_OFC_SETTING, 0x20); // 0010 0000
  
  // Enable FOC for x axis
  acc_write(AM_OFC_CTRL, 0x20);
  while(!(acc_read(AM_OFC_CTRL)& 0x10));
  
  // Enable FOC for y axis
  acc_write(AM_OFC_CTRL, 0x40);
  while(!(acc_read(AM_OFC_CTRL)& 0x10));

  // Enable FOC for z axis
  acc_write(AM_OFC_CTRL, 0x60);
  while(!(acc_read(AM_OFC_CTRL)& 0x10));*/

  // Set G range to 0x03 = +/- 2g, 0x08 = +/- 8g, needs to be in 2g for fast offset compnesation
  acc_write(AM_PMU_RANGE, 0x08);

  get_acc_settings();
}

void BMX055::gyr_init()
{
  // Reset gyroscope and wait 1ms
  gyr_write(G_BGW_SOFT_RESET, 0xB6);
  delay(4);
  //delay(2);

  // Set range to 0x03 = +/- 250 deg/s, 0x02 =+/- 500deg/s
  gyr_write(G_RANGE, 0x02);
  //gyr_write(G_RANGE, 0x02);

  // 0x05 = 100Hz ODR and lp-filter at 12Hz. 0x04 = 200Hz lpf at 23Hz. 0x03 = 400Hz lpf at 47Hz
  gyr_write(G_BW, 0x04); //05

  //RATE_HBW
  gyr_write(G_RATE_HBW, 0x00); // Enables data shadowinf and filtered output

  // 64 samples @ 100hZ and enables it for x
  gyr_write(G_A_FOC, 0x5F);

  //maybe read 0x1B bit 7 should be 0
  delay(640);

  get_gyr_settings();
}

void BMX055::mag_init()
{
  // Reset magnetometer and wait 1.8ms
  mag_write(MAG_SOFT_RESET, 0x83);
  delay(2);

  // Data rate 10hZ, normal mode
  mag_write(MAG_MODR, 0x00);

  // 9 repetitions for x and y,  MAG_REPXY*2 +1
  mag_write(MAG_REPXY, 0x04);

  // 16 repetitions for z,  MAG_REPZ*1 +1
  mag_write(MAG_REPZ, 0x0E);

  // Reading trime register, used to compensate compass values
  mag_read_trim_registers();

#if CALIBRATE_YAW
#if DEBUG
  Serial.println("Beginning yaw calibration in 1 seconds");
  delay(1000);
#endif
  mag_yaw_calibration(25);

#if DEBUG
  Serial.println("Finished calibration");
#endif
#endif

  get_mag_settings();
}

void BMX055::mag_read_trim_registers()
{
  // TODO: RECHECK if TWO COMPLEMENT IS NEEDE
  uint16_t temp_msb = 0;

  magnetometer.dig_x1 = twos_comp(mag_read(BMM150_DIG_X1), 8); // twos complement
  magnetometer.dig_y1 = twos_comp(mag_read(BMM150_DIG_Y1), 8); // twos complement
  magnetometer.dig_x2 = twos_comp(mag_read(BMM150_DIG_X2), 8); // twos complement
  magnetometer.dig_y2 = twos_comp(mag_read(BMM150_DIG_Y2), 8); // twos complement

  temp_msb = mag_read(BMM150_DIG_Z1_LSB+1) << 8;
  magnetometer.dig_z1 = twos_comp(temp_msb | mag_read(BMM150_DIG_Z1_LSB), 16); // twos complement

  temp_msb = mag_read(BMM150_DIG_Z2_LSB+1) << 8;
  magnetometer.dig_z2 = twos_comp(temp_msb | mag_read(BMM150_DIG_Z2_LSB), 16); // twos complement

  temp_msb = mag_read(BMM150_DIG_Z3_LSB+1) << 8;
  magnetometer.dig_z3 = twos_comp(temp_msb | mag_read(BMM150_DIG_Z3_LSB), 16); // twos complement

  temp_msb = mag_read(BMM150_DIG_Z4_LSB+1) << 8;
  magnetometer.dig_z4 = twos_comp(temp_msb | mag_read(BMM150_DIG_Z4_LSB), 16); // twos complement

  //magnetometer.dig_xy1 = trim_xy1xy2[9];
  magnetometer.dig_xy1 = mag_read(BMM150_DIG_XY1); // not twos complement

  //magnetometer.dig_xy2 = (int8_t)trim_xy1xy2[8];
  magnetometer.dig_xy2 = twos_comp(mag_read(BMM150_DIG_XY2), 8); // twos complement

  temp_msb = mag_read(BMM150_DIG_XYZ1_LSB+1) << 8;
  magnetometer.dig_xyz1 = temp_msb | mag_read(BMM150_DIG_XYZ1_LSB); // not twos complement
}

void BMX055::get_acc_data()
{
    int16_t xd = acc_read(AM_ACCD_X_LSB+1)<<4;
    xd |= (acc_read(AM_ACCD_X_LSB) & 0xF0)>>4;
    xd = twos_comp(xd, 12);
    accelerometer.x = (float) xd;

    int16_t yd = acc_read(AM_ACCD_Y_LSB+1)<<4;
    yd |= (acc_read(AM_ACCD_Y_LSB) & 0xF0)>>4;
    yd = twos_comp(yd, 12);
    accelerometer.y = (float) yd;

    int16_t zd = acc_read(AM_ACCD_Z_LSB+1)<<4;
    zd |= (acc_read(AM_ACCD_Z_LSB) & 0xF0)>>4;
    zd = twos_comp(zd, 12);
    accelerometer.z = (float) zd;
}

void BMX055::get_gyr_data()
{
    int16_t xd = gyr_read(G_RATE_X_LSB+1)<<8;
    xd |= gyr_read(G_RATE_X_LSB);
    xd = twos_comp(xd, 16);
    gyroscope.x = (float) xd;

    int16_t yd = gyr_read(G_RATE_Y_LSB+1)<<8;
    yd |= gyr_read(G_RATE_Y_LSB);
    yd = twos_comp(yd, 16);
    gyroscope.y = (float) yd;

    int16_t zd = gyr_read(G_RATE_Z_LSB+1)<<8;
    zd |= gyr_read(G_RATE_Z_LSB);
    zd = twos_comp(zd, 16);
    gyroscope.z = (float) zd;
}

void BMX055::get_mag_data()
{  
  int16_t xd = mag_read(MAG_DATAX_LSB+1)<<5;
  xd |= mag_read(MAG_DATAX_LSB)>>3;
  xd = twos_comp(xd, 13);

  int16_t yd = mag_read(MAG_DATAY_LSB+1)<<5;
  yd |= mag_read(MAG_DATAY_LSB)>>3;
  yd = twos_comp(yd, 13);

  int16_t zd = mag_read(MAG_DATAZ_LSB+1)<<7;
  zd |= mag_read(MAG_DATAZ_LSB)>>1;
  zd = twos_comp(zd, 15);

  uint16_t rd = mag_read(MAG_RHALL_LSB+1)<<6;
  rd |= mag_read(MAG_RHALL_LSB)>>2;


  if(mag_cal == USE_MAG_CALIBRATION)
  {
    double x = mag_compensate_x(xd, rd) - magnetometer.x_offset;
    double y = mag_compensate_y(yd, rd) - magnetometer.y_offset;
    double z = mag_compensate_z(zd, rd) - magnetometer.z_offset;
    
    magnetometer.x = x*magnetometer.soft_iron[0] + y*magnetometer.soft_iron[1] + z*magnetometer.soft_iron[2];
    magnetometer.y = x*magnetometer.soft_iron[3] + y*magnetometer.soft_iron[4] + z*magnetometer.soft_iron[5];
    magnetometer.z = x*magnetometer.soft_iron[6] + y*magnetometer.soft_iron[7] + z*magnetometer.soft_iron[8];
  }
  else if(mag_cal == USE_MAG_RAWDATA)
  {
    magnetometer.x = mag_compensate_x(xd, rd);
    magnetometer.y = mag_compensate_y(yd, rd);
    magnetometer.z = mag_compensate_z(zd, rd);
  }
}

void BMX055::mag_yaw_calibration(uint8_t samples)
{
  // Initialize all temp variables to the same value
  double yaw_avg = 0;
  
  for(uint8_t i = 0; i < samples; i++)
  {
    get_mag_data();
    double xh = magnetometer.x;
    double yh = magnetometer.y;
    yaw_avg += atan2(yh, xh);
#if DEBUG
    Serial.print(".");
#endif
    //delay(1000.0/((float)magnetometer.odr));
    delay(101);
  }
#if DEBUG
  Serial.println("");
#endif

  magnetometer.yaw_offset = yaw_avg/((float) samples) - MAG_TARGET;
}

float BMX055::mag_compensate_x(int16_t mag_data_x, uint16_t data_rhall)
{
  float retval = 0;
  float process_comp_x0;
  float process_comp_x1;
  float process_comp_x2;
  float process_comp_x3;
  float process_comp_x4;
  
  if ((mag_data_x != -4096) && (data_rhall != 0) && (magnetometer.dig_xyz1 != 0)) 
  {
    process_comp_x0 = (((float)magnetometer.dig_xyz1) * 16384.0f/data_rhall);
    retval = (process_comp_x0 - 16384.0f);
    process_comp_x1 = ((float)magnetometer.dig_xy2) * (retval * retval / 268435456.0f);
    process_comp_x2 = process_comp_x1 + retval * ((float)magnetometer.dig_xy1) / 16384.0f;
    process_comp_x3 = ((float)magnetometer.dig_x2) + 160.0f;
    process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
    retval = ((process_comp_x4 / 8192.0f) + (((float)magnetometer.dig_x1) * 8.0f)) / 16.0f;
  }
  else
  {
    retval = 0;
  }

  return retval;
}

float BMX055::mag_compensate_y(int16_t mag_data_y, uint16_t data_rhall)
{
  float retval = 0;
  float process_comp_y0;
  float process_comp_y1;
  float process_comp_y2;
  float process_comp_y3;
  float process_comp_y4;
  
  if ((mag_data_y != -4096) && (data_rhall != 0) && (magnetometer.dig_xyz1 != 0)) 
  {
    process_comp_y0 = (((float)magnetometer.dig_xyz1) * 16384.0f/data_rhall);
    retval = (process_comp_y0 - 16384.0f);
    process_comp_y1 = ((float)magnetometer.dig_xy2) * (retval * retval / 268435456.0f);
    process_comp_y2 = process_comp_y1 + retval * ((float)magnetometer.dig_xy1) / 16384.0f;
    process_comp_y3 = ((float)magnetometer.dig_y2) + 160.0f;
    process_comp_y4 = mag_data_y * ((process_comp_y2 + 256.0f) * process_comp_y3);
    retval = ((process_comp_y4 / 8192.0f) + (((float)magnetometer.dig_y1) * 8.0f)) / 16.0f;
  }
  else
  {
    retval = 0;
  }

  return retval;
}

float BMX055::mag_compensate_z(int16_t mag_data_z, uint16_t data_rhall)
{
  float retval = 0;
  float process_comp_z0;
  float process_comp_z1;
  float process_comp_z2;
  float process_comp_z3;
  float process_comp_z4;
  float process_comp_z5;

  if ((mag_data_z != -16384) && (magnetometer.dig_z2 != 0) && (magnetometer.dig_z1 != 0) && (magnetometer.dig_xyz1 != 0) && (data_rhall != 0))
  {
    process_comp_z0 = ((float)mag_data_z) - ((float)magnetometer.dig_z4);
    process_comp_z1 = ((float)data_rhall) - ((float)magnetometer.dig_xyz1);
    process_comp_z2 = (((float)magnetometer.dig_z3) * process_comp_z1);
    process_comp_z3 = ((float)magnetometer.dig_z1) * ((float)data_rhall) / 32768.0f;
    process_comp_z4 = ((float)magnetometer.dig_z2) + process_comp_z3;
    process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
    retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
  }
  else
  {
    retval = 0;
  }

  return retval;
}

void BMX055::get_acc_settings()
{
  accelerometer.chipid = acc_read(AM_CHIPID_REG);
  accelerometer.range = acc_read(AM_PMU_RANGE);
  switch(accelerometer.range)
  {
    case 3:
      accelerometer.range = 2;
      break;
    case 5:
      accelerometer.range = 4;
      break;
    case 8:
      accelerometer.range = 8;
      break;
    case 12:
      accelerometer.range = 16;
      break;
    default:
      accelerometer.range = 0;
      break;
  }
  accelerometer.res = ((float) accelerometer.range)*9.81*2.0/pow(2,12);

  uint8_t bw = acc_read(AM_PMU_BW);
  switch(bw)
  {
    case 0x09:
      accelerometer.bandwidth = 15.63;
      break;
    case 0x0A:
      accelerometer.bandwidth = 31.25;
      break;
    case 0x0B:
      accelerometer.bandwidth = 62.5;
      break;
    case 0x0C:
      accelerometer.bandwidth = 125;
      break;
    case 0x0D:
      accelerometer.bandwidth = 250;
      break;
    case 0x0E:
      accelerometer.bandwidth = 500;
      break;
    default:
      if(bw >= 15)
      {
        accelerometer.bandwidth = 1000;
      }
      else if(bw <= 8)
      {
        accelerometer.bandwidth = 7.81;
      }
      break;
  }
  accelerometer.sample_time = 1000.0/(accelerometer.bandwidth*2.0);

  accelerometer.x_offset = (float) twos_comp(acc_read(AM_OFC_OFFSET_X),8);
  accelerometer.y_offset = (float) twos_comp(acc_read(AM_OFC_OFFSET_Y),8);
  accelerometer.y_offset = (float) twos_comp(acc_read(AM_OFC_OFFSET_Z),8);   

#if DEBUG
  Serial.print("Chip ID = ");
  Serial.println(accelerometer.chipid);
  
  Serial.print("Accelerometer range of +/- ");
  Serial.print(accelerometer.range);
  Serial.print("g with a resolution of ");
  Serial.print(accelerometer.res, 6);
  Serial.println(" m/s^2");
  
  Serial.print("Accelerometer bandwidth of ");
  Serial.print(accelerometer.bandwidth);
  Serial.print("Hz and smpling time ");
  Serial.print(accelerometer.sample_time);
  Serial.println(" ms");
  
  Serial.print("X offset is ");
  Serial.print(accelerometer.x_offset);
  Serial.println(" m/s^2");
  Serial.print("Y offset is ");
  Serial.print(accelerometer.y_offset);
  Serial.println(" m/s^2");
  Serial.print("Z offset is ");
  Serial.print(accelerometer.z_offset);
  Serial.println(" m/s^2");
#endif
}

void BMX055::get_gyr_settings()
{
  gyroscope.chipid = gyr_read(G_CHIPID_REG);
  gyroscope.range = gyr_read(G_RANGE);
  switch(gyroscope.range)
  {
    case 0:
      gyroscope.range = 2000;
      break;
    case 1:
      gyroscope.range = 1000;
      break;
    case 2:
      gyroscope.range = 500;
      break;
    case 3:
      gyroscope.range = 250;
      break;
    case 4:
      gyroscope.range = 125;
      break;
    default:
      gyroscope.range = 0;
      break;
  }
  gyroscope.res = ((float) gyroscope.range)*2.0/pow(2,16);

  uint8_t bw = gyr_read(G_BW) & 0x07;
  switch(bw)
  {
    case 0:
      gyroscope.bandwidth = 523;
      gyroscope.sample_time = 0.5;
      break;
    case 1:
      gyroscope.bandwidth = 230;
      gyroscope.sample_time = 0.5;
      break;
    case 2:
      gyroscope.bandwidth = 116;
      gyroscope.sample_time = 1;
      break;
    case 3:
      gyroscope.bandwidth = 47;
      gyroscope.sample_time = 2.5;
      break;
    case 4:
      gyroscope.bandwidth = 23;
      gyroscope.sample_time = 5;
      break;
    case 5:
      gyroscope.bandwidth = 12;
      gyroscope.sample_time = 10;
      break;
    case 6:
      gyroscope.bandwidth = 64;
      gyroscope.sample_time = 5;
      break;
    case 7:
      gyroscope.bandwidth = 32;
      gyroscope.sample_time = 10;
      break;
  }

  uint16_t xo = gyr_read(G_OFC2) << 4; // x bits 11:4
  xo |= (gyr_read(G_OFC1) & 0xC0)>>4; // x bits 3:2
  xo |= (gyr_read(G_TRIM_GP0) & 0x0C) >> 2; // x bits 1:0
  gyroscope.x_offset = (float) twos_comp(xo, 12);

  uint16_t yo = gyr_read(G_OFC3) << 4; // y bits 11:4
  yo |= (gyr_read(G_OFC1) & 0x38) >> 2; // y bits 3:1
  yo |= (gyr_read(G_TRIM_GP0) & 0x02) >> 1; // y bits 0
  gyroscope.y_offset = (float) twos_comp(yo, 12);

  uint16_t zo = gyr_read(G_OFC4) << 4; // z bits 11:4
  zo |= (gyr_read(G_OFC1) & 0x07) << 1; // z bits 3:1
  zo |= gyr_read(G_TRIM_GP0) & 0x01; // z bits 0
  gyroscope.z_offset = (float) twos_comp(zo, 12);
   
#if DEBUG
  Serial.print("Chip ID = ");
  Serial.println(gyroscope.chipid);
  
  Serial.print("Gyroscope range of +/- ");
  Serial.print(gyroscope.range);
  Serial.print("deg/s with a resolution of ");
  Serial.print(gyroscope.res, 6);
  Serial.println(" deg/s");
  
  Serial.print("Gyroscope bandwidth of ");
  Serial.print(gyroscope.bandwidth);
  Serial.print("Hz and smpling time ");
  Serial.print(gyroscope.sample_time);
  Serial.println(" ms");
  
  Serial.print("X offset is ");
  Serial.print(gyroscope.x_offset);
  Serial.println(" deg/s");
  Serial.print("Y offset is ");
  Serial.print(gyroscope.y_offset);
  Serial.println(" deg/s");
  Serial.print("Z offset is ");
  Serial.print(gyroscope.z_offset);
  Serial.println(" deg/s");
#endif
}

void BMX055::get_mag_settings()
{ 
  magnetometer.chipid = mag_read(MAG_CHIPID_REG);
  
  uint8_t modr = mag_read(MAG_MODR);
  switch(modr & 0x38)
  {
    case 0:
      magnetometer.odr = 10;
      break;
    case 1:
      magnetometer.odr = 2;
      break;
    case 2:
      magnetometer.odr = 6;
      break;
    case 3:
      magnetometer.odr = 8;
      break;
    case 4:
      magnetometer.odr = 15;
      break;
    case 5:
      magnetometer.odr = 20;
      break;
    case 6:
      magnetometer.odr = 25;
      break;
    case 7:
      magnetometer.odr = 30;
      break;
  }
  switch(modr & 0x06)
  {
    case 0:
      magnetometer.opmode = "Normal";
      break;
    case 1:
      magnetometer.opmode = "Forced";
      break;
    case 2:
      magnetometer.opmode = "Reserved";
      break;
    case 3:
      magnetometer.opmode = "Sleep";
      break;
  }

  magnetometer.repetitions_xy = mag_read(MAG_REPXY)*2 + 1;
  magnetometer.repetitions_z = mag_read(MAG_REPZ) + 1;
   
#if DEBUG
  Serial.print("Chip ID = ");
  Serial.println(magnetometer.chipid);
  
  Serial.print("Magnetometer resolution of ");
  Serial.print(magnetometer.res, 4);
  Serial.println(" uT");
  
  Serial.print("Output data rate is ");
  Serial.print(magnetometer.odr);
  Serial.println("Hz");

  Serial.print("Magnetometer running in ");
  Serial.print(magnetometer.opmode);
  Serial.println(" mode");

  Serial.print("Set to ");
  Serial.print(magnetometer.repetitions_xy);
  Serial.println(" x and y repetitions");

  Serial.print("Set to ");
  Serial.print(magnetometer.repetitions_z);
  Serial.println(" z repetitions");

  Serial.print("x offset is ");
  Serial.println(magnetometer.x_offset);

  Serial.print("y offset is ");
  Serial.println(magnetometer.y_offset);

  Serial.print("z offset is ");
  Serial.println(magnetometer.z_offset);

  Serial.print("Yaw offset is ");
  Serial.print(magnetometer.yaw_offset * 180.0/3.141592653589793);
  Serial.println(" degrees");

#if 0
  Serial.println(magnetometer.dig_x1);
  Serial.println(magnetometer.dig_y1);
  Serial.println(magnetometer.dig_x2);
  Serial.println(magnetometer.dig_y2);
  Serial.println(magnetometer.dig_z1);
  Serial.println(magnetometer.dig_z2);
  Serial.println(magnetometer.dig_z3);
  Serial.println(magnetometer.dig_z4);
  Serial.println(magnetometer.dig_xy1);
  Serial.println(magnetometer.dig_xy2);
  Serial.println(magnetometer.dig_xyz1);
#endif
#endif
}

int16_t BMX055::twos_comp(uint16_t val, uint8_t bits)
{
    /*if((val & (1 << (bits - 1))) != 0)
        val = val - (1 << bits);
    return val;*/
    int mask = pow(2, bits-1);
    return -(val & mask) + (val & ~mask);
}

uint8_t BMX055::acc_read(uint8_t reg)
{
  uint8_t rx_data[1];
  i2cget(Addr_Accel, reg, rx_data, 1);
  return rx_data[0];
}

uint8_t BMX055::acc_write(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[1];
    tx_data[0] = reg;
    tx_data[1] = data;
    return i2cset(Addr_Accel, tx_data, 2, true);
}

uint8_t BMX055::gyr_read(uint8_t reg)
{
  uint8_t rx_data[1];
  i2cget(Addr_Gyro, reg, rx_data, 1);
  return rx_data[0];
}

uint8_t BMX055::gyr_write(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[1];
    tx_data[0] = reg;
    tx_data[1] = data;
    return i2cset(Addr_Gyro, tx_data, 2, true);
}

uint8_t BMX055::mag_read(uint8_t reg)
{
  uint8_t rx_data[1];
  i2cget(Addr_magnet, reg, rx_data, 1);
  return rx_data[0];
}

uint8_t BMX055::mag_write(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[1];
    tx_data[0] = reg;
    tx_data[1] = data;
    return i2cset(Addr_magnet, tx_data, 2, true);
}
