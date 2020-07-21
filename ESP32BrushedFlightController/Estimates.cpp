#include "Estimates.h"

uint16_t missing_samples = 0;
float accSum = 0;
float accTimeSum = 0;
uint16_t accSumCount = 0;

int32_t baroPressure = 0;
int32_t baroTemperature = 0;
uint32_t baroPressureSum = 0;
uint16_t missing_alt_samples = 0;
int32_t BaroAlt = 0;

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

void get_gyr_lpf()
{
  gyroLPF[0]  = gyroLPF[0]*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
  gyroLPF[1]  = gyroLPF[1]*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
  gyroLPF[2]  = gyroLPF[2]*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;
}

// cfg.baro_tab_size = 21;
//#define BARO_TAB_SIZE_MAX   48
void Baro_Common(void)
{
    static int32_t baroHistTab[48];
    static int baroHistIdx;
    int indexplus1;

    indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == 21)
        indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;
}

uint8_t get_baro_data()
{
  static int state = 0;
  static uint32_t previousT;
  uint32_t currentT = micros();
  float dt = currentT - previousT;

  if (dt < altimeter.ct*1000.0)
    return 0;
  previousT = currentT;

  uint32_t readRawTemperature(void);
      uint32_t readRawPressure(void);

  if(!state)
  {
    altimeter.readRawTemperature();
    altimeter.requestPressure();
    Baro_Common(); // LPF using Rolling memory
    state = 1;
    return 1;
  }
  else//if(state)
  {
    altimeter.readRawPressure();
    altimeter.requestTemperature();
    altimeter.calculatePressure(&baroPressure, &baroTemperature);
    state = 0;
    return 2;
  }
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

  accSum += applyDeadband(acczSmooth, imu.ACC_1G/32);
  accTimeSum += dt;
  accSumCount++;
}

#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)

uint8_t altitude_estimation()
{
  static int32_t baroGroundPressure = 0;
  static int32_t baroGroundAltitude = 0;

  int32_t BaroAlt_tmp;
  static int32_t lastBaroAlt;
  int32_t baroVel;
  static float vel = 0.0f;  
  static float lpf_vel = 0.0f;
  static float accAlt = 0.0f;
  static float accZ_tmp = 0;
  static float accZ_old = 0.0f;
  
  static uint32_t previousT;
  uint32_t currentT = micros();
  float dt = currentT - previousT;

  // Update with a constant period
  if (dt < UPDATE_INTERVAL)
    return 0;
  previousT = currentT;

  if(calibrate_alt)
  {
    missing_alt_samples = BARO_CAL_SAMPLES;
    calibrate_alt = 0;
  }

  if(missing_alt_samples > 0)
  {

    baroGroundPressure = 0.9*baroGroundPressure + 0.1*(baroPressureSum / (21 - 1));
    baroGroundAltitude = (1.0f - powf(baroGroundPressure / 101325.0f, 0.190295f)) * 4433000.0f;

    vel = 0;
    accAlt = 0;
    EstAlt = 0;
    missing_alt_samples--;
  }

  BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / (21 - 1)) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
  BaroAlt_tmp -= baroGroundAltitude;
  //BaroAlt = lrintf((float)BaroAlt * 0.6 + (float)BaroAlt_tmp * 0.4); // additional LPF to reduce baro noise
  BaroAlt = lrintf((float)BaroAlt * 0.9 + (float)BaroAlt_tmp * 0.1); // additional LPF to reduce baro noise

  float acc_dt = accTimeSum * 1e-6f;
  accZ_tmp = 0.7*accZ_tmp + 0.3*(float)accSum / (float)accSumCount;

  float vel_acc  = accZ_tmp * imu.accelerometer.res*100.0 * acc_dt; // vel diff in cm/s

  //accAlt += (vel_acc * 0.5f) * acc_dt + vel * acc_dt;   // integrate velocity to get distance (x= a/2 * t^2)
  //accAlt = accAlt * 0.965 + (float)BaroAlt * 0.035;      // complementary filter for altitude estimation (baro & acc)

  accAlt = (vel_acc * 0.5f) * acc_dt + vel * acc_dt;   // integrate velocity to get distance (x= a/2 * t^2)
  EstAlt = (EstAlt + accAlt) * 0.9 + (float)BaroAlt * 0.1;      // complementary filter for altitude estimation (baro & acc)

  vel += vel_acc;

  //accSum_reset();
  accSum = 0;
  accTimeSum = 0;
  accSumCount = 0;

  baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dt;
  lastBaroAlt = BaroAlt;

  baroVel = constrain(baroVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
  baroVel = applyDeadband(baroVel, 10);         // to reduce noise near zero
  lpf_vel = 0.9*lpf_vel + 0.1*baroVel;

  vel = vel * 0.9 + lpf_vel * 0.1;
  //vel = vel * 0.985 + baroVel * 0.015;
  //lpf_vel = 0.7*lpf_vel + 0.3*vel;
  int32_t vel_tmp = lrintf(vel);
  //Serial.print(BaroAlt);
  //Serial.print(" ");
  //Serial.println(EstAlt);

  //Serial.print(baroVel);
  //Serial.print(" ");
  //Serial.println(lpf_vel);
  
  //Serial.print(accZ_tmp);
  //Serial.print(" ");
  //Serial.println(vel);

  int32_t error;
  int32_t setVel;
  uint8_t alt_p = 25;//50;
  uint8_t vel_kp = 80;//50;//120;
  uint8_t vel_ki = 0;//45;
  uint8_t vel_kd = 200;//1;
  
  if(roll*180/PI < 60 && pitch*180/PI < 60)
  {
    if(true)
    {
      error = constrain(AltitudeSetpoint-EstAlt, -500, 500);
      error = applyDeadband(error, 10);
      setVel = constrain((alt_p * error / 128), -300, +300); // limit velocity to +/- 3 m/s
      //Serial.println(error);
    }
    else
    {
      setVel = 0;
    }
    
    // Proportianl term
    error = setVel - vel_tmp;
    BaroPID = constrain(vel_kp*error/32, -300, 300);

    // Integral term
    errorVelocityI += (vel_ki * error);
    errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
    BaroPID += errorVelocityI / 8196;

    // Derivative term
    BaroPID -= constrain(vel_kd * (accZ_tmp + accZ_old) / 512, -150, 150);

    //Serial.print(BaroAlt);
    //Serial.print(" ");
    //Serial.println(EstAlt);
    //Serial.println(vel_tmp);
  }
  else
  {
    BaroPID = 0;
  }

  accZ_old = accZ_tmp;
  return 1;
}
