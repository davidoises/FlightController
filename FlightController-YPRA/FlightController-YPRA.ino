#include "RollingMemory.h"
#include "BMX055.h"
#include "SensorFusion.h"
#include "MS5611.h"

// UI Control variables
double eStop = 1;
double throttle = 1000.0;
double kr = 0; //40
double ka = 0; // 4.7

double kp_roll = 11;
double ki_roll = 0;
double kd_roll = 0.8;

double kp_pitch = 11;
double ki_pitch = 0;
double kd_pitch = 0.8;

double kp_yaw = 5.0;
double ki_yaw = 0;
double kd_yaw = 0.1;

double kp_vel_z = 0;//1090.0;
double ki_vel_z = 0;
double kd_vel_z = 0;

double roll_setpoint = 0;//0.5;
double pitch_setpoint = 0;
double yaw_setpoint = 0;
double alt_setpoint = 0;

#include "ui_conf.h"

#define TXD2 9
#define RXD2 15

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10
#define SERVO1_X 13
#define SERVO1_Y 12
#define SERVO2_X 26
#define SERVO2_Y 25
#define ADCA 37
#define ADCB 38
#define ADCC 34
#define ADCD 35
#define LASER1 0
#define LASER2 2

// setting PWM properties
#define FREQ 250
#define RESOLUTION 12
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3
#define ledChannelS1X 4
#define ledChannelS1Y 5
#define ledChannelS2X 6
#define ledChannelS2Y 7

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((double)FREQ)
#define MS_TO_PWM(x) ((double)x)*((double)MAX_PWM)/(((double)MAX_PERIOD)*1000.0f)

// Attitude sampling
#define ORIENTATION_SAMPLING 0.004f
#define ALTITUDE_SAMPLING 0.009f

// Class objects for data acquisition and sensor fusion
MS5611 ms5611;
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
SensorFusion orientation;

// timer variables
volatile uint8_t update_orientation;
volatile uint8_t tp_counter = 0;
volatile uint8_t alt_counter = 0;
volatile uint8_t update_altitude;
hw_timer_t * orientation_timer = NULL;

// Timer ISR
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
  // Update altitude every 3 orientation samples (ORIENTATION SAMPLING *3)
  alt_counter++;
  if(alt_counter == 3)
  {
    update_altitude = 1;
    alt_counter = 0;
  }
}

// Angular measurement variables
unsigned long orientation_prev_time = 0;
double roll_rate = 0;
double pitch_rate = 0;
double yaw_rate = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

// Altitude acceleration measurements
double acc_z = 0;
double prev_acc_z = 0;
double initial_acc = 0;
double lpf_acc_z = 0;
RollingMemory acc_av;

// Altitude measurement variables
unsigned long altitude_prev_time = 0;
uint32_t raw_temp = 0;
double unfiltered_pressure = 0;
double pressure = 0;
double prev_pressure = 0;
double initial_alt = 0;
RollingMemory pres_av;

// Used to get calibration samples
unsigned long initial_time = 0;
uint8_t sampled_calibration = 0;

// Complementary kalman
double pos_z =0;
double vel_Z =0;
double k2 = 35.0/20.0; // if numerator is bigger it means noisier accelerometer
double k1 = sqrt((2*k2));

// Orientation PID calculation variables
double roll_integral = 0;
double roll_prev_error = 0;
double pitch_integral = 0;
double pitch_prev_error = 0;
double yaw_integral = 0;
double yaw_prev_error = 0;

// Altitude PID calculation variables
double vel_z_integral = 0;
double vel_z_pid = 0;

// Serial packages sincronization
int serial_counter = 0;

void setup() {

  // UART 2  initilization
  Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  
  // UART 1 Initialization
  //Serial.begin(115200);
  Serial.begin(500000);
  //Serial.begin(1000000);
  delay(100);
  Serial.println("\r\n");
  delay(100);

  // I2C initialization
  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);
  //Wire1.begin(9, 15, 400000); // SDA = 9, SCL = 15

  // ESC initialization, all set to 1000ms pulse
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA); // negative torque
  ledcWrite(ledChannelA, MS_TO_PWM(1000));

  ledcSetup(ledChannelB, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, ledChannelB); // negative torque
  ledcWrite(ledChannelB, MS_TO_PWM(1000));

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC); // Positive toruqe
  ledcWrite(ledChannelC, MS_TO_PWM(1000));

  ledcSetup(ledChannelD, FREQ, RESOLUTION);
  ledcAttachPin(PWMD, ledChannelD); // Positive toruqe
  ledcWrite(ledChannelD, MS_TO_PWM(1000));

  ledcSetup(ledChannelS1X, FREQ, RESOLUTION);
  ledcAttachPin(SERVO1_X, ledChannelS1X); // Positive toruqe
  ledcWrite(ledChannelS1X, MS_TO_PWM(1000));

  ledcSetup(ledChannelS1Y, FREQ, RESOLUTION);
  ledcAttachPin(SERVO1_Y, ledChannelS1Y); // Positive toruqe
  ledcWrite(ledChannelS1Y, MS_TO_PWM(1000));

  ledcSetup(ledChannelS2X, FREQ, RESOLUTION);
  ledcAttachPin(SERVO2_X, ledChannelS2X); // Positive toruqe
  ledcWrite(ledChannelS2X, MS_TO_PWM(1000));

  ledcSetup(ledChannelS2Y, FREQ, RESOLUTION);
  ledcAttachPin(SERVO2_Y, ledChannelS2Y); // Positive toruqe
  ledcWrite(ledChannelS2Y, MS_TO_PWM(1000));

  pinMode(LASER1, OUTPUT);
  pinMode(LASER2, OUTPUT);
  digitalWrite(LASER1, LOW);
  digitalWrite(LASER2, LOW);

  // Start user interface while ESCs set up
  #if WIFI_GUI
  Blynk.begin(auth, ssid, pass);
  #else
  Blynk.begin(auth);
  #endif

  // IMU sensors initialization
  imu.acc_init();
  imu.gyr_init();
  //imu.mag_init();
  for(int i = 0; i < 200; i++)
  {
    imu.get_acc_data();
    delay(5);
  }
  double initial_acc_roll = 0;
  double initial_acc_pitch = 0;
  for(int i = 0; i < 100; i++)
  {
    imu.get_acc_data();
    double ax = imu.accelerometer.x;
    double ay = imu.accelerometer.y;
    double az = imu.accelerometer.z;
    initial_acc_roll += atan2(ay, sqrt(pow(ax, 2) + pow(az, 2)));
    initial_acc_pitch += atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2)));
    initial_acc += az*imu.accelerometer.res;
    delay(4);
  }
  initial_acc_roll /= 100;
  initial_acc_pitch /= 100;
  initial_acc /= 100.0;
  acc_av.samples = 20.0;

  // Altitude sensor initialization
  ms5611.begin();
  ms5611.setOversampling(MS5611_ULTRA_HIGH_RES);
  // Get the first measurements for future calculations
  ms5611.requestTemperature();
  delay(ALTITUDE_SAMPLING*1000.0);
  raw_temp = ms5611.readRawTemperature();
  ms5611.requestPressure();
  delay(ALTITUDE_SAMPLING*1000.0);

  // Orientation timer
  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);
  //orientation.set_yaw_offset(imu.magnetometer.yaw_offset);
  orientation.init(initial_acc_roll, initial_acc_pitch, MAG_TARGET);

  /*while(throttle < 1100)
  {
    Blynk.run();
  }*/
  
  // Start timer
  timerAlarmEnable(orientation_timer);
  
  orientation_prev_time = millis();
  altitude_prev_time = millis();
  initial_time = millis();
}

void loop() {
  
  Blynk.run();

  // Sampled calibration flag: indicates inintial values for some sensores have been stored
  if(sampled_calibration < 2)
  {
    if(sampled_calibration == 0 && (millis()-initial_time) > 2500)
    {
      // Calibration value for altitude
      initial_alt = pressure;
      sampled_calibration = 1;
    }
    if(sampled_calibration < 2 && (millis()-initial_time) > 3000)
    {
      sampled_calibration = 2;
    }
  }

  if(ui_callback)
  {    
    roll_integral = 0;
    roll_prev_error = 0;
    pitch_integral = 0;
    pitch_prev_error = 0;
    yaw_integral = 0;
    yaw_prev_error = 0;
    ui_callback = 0;
  }
  if(alt_callback)
  {
    if(alt_hold)
    {
      //roll_setpoint = roll*180/PI;
      //pitch_setpoint = pitch*180/PI;
      alt_setpoint = pos_z;
      vel_z_integral = 0;
    }
    else
    {
      vel_z_pid = 0;
    }
    
    alt_callback = 0;
  }

  if(eStop)
  {
    ledcWrite(ledChannelA, MS_TO_PWM(1000));
    ledcWrite(ledChannelB, MS_TO_PWM(1000));
    ledcWrite(ledChannelC, MS_TO_PWM(1000));
    ledcWrite(ledChannelD, MS_TO_PWM(1000));
    ledcWrite(ledChannelS1X, MS_TO_PWM(1000));
    ledcWrite(ledChannelS1Y, MS_TO_PWM(1000));
    ledcWrite(ledChannelS2X, MS_TO_PWM(1000));
    ledcWrite(ledChannelS2Y, MS_TO_PWM(1000));
    digitalWrite(LASER1, LOW);
    digitalWrite(LASER2, LOW);
  }
  
  if(update_orientation)
  {
    // Loop time for pid and time integration
    unsigned long current_time = millis();
    double dt = (current_time - orientation_prev_time)/1000.0;
    orientation_prev_time = millis();

    // Imu data collection and attitude sensor fusion
    imu.get_gyr_data();
    imu.get_acc_data();
    //imu.get_mag_data();
    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);


    // Angular position measurement
    roll = orientation.get_roll();//0.7*roll + 0.3*orientation.get_roll();
    pitch = orientation.get_pitch();//0.7*pitch + 0.3*orientation.get_pitch();
    
    // Angular rate measurements
    roll_rate = roll_rate*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
    pitch_rate = pitch_rate*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
    yaw_rate = yaw_rate*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;

    // Vertical acceleration calculation and filtering
    prev_acc_z = acc_z;
    acc_z = (imu.accelerometer.z*cos(pitch)*cos(roll) - imu.accelerometer.x*sin(pitch) + imu.accelerometer.y*cos(pitch)*sin(roll))*imu.accelerometer.res - initial_acc;
    acc_av.sum -= acc_av.memory[acc_av.index];
    acc_av.memory[acc_av.index] = acc_z;
    acc_av.sum += acc_av.memory[acc_av.index];
    acc_av.index++;
    if(acc_av.index == acc_av.samples) acc_av.index = 0;
    double acc_z_av = acc_av.sum/acc_av.samples;
    lpf_acc_z = 0.65*lpf_acc_z + 0.36*acc_z_av; // This low-pass-filtered signal will be used for PI+D velocity controller

    double acc_x = (imu.accelerometer.x*cos(pitch) + imu.accelerometer.z*cos(roll)*sin(pitch) + imu.accelerometer.y*sin(pitch)*sin(roll))*imu.accelerometer.res;
    double acc_y = (imu.accelerometer.y*cos(roll) - imu.accelerometer.z*sin(roll))*imu.accelerometer.res;
    
    // Vertical channel velocity and position complementary-kalman filter
    double dz = prev_pressure - pos_z;
    if(sampled_calibration != 2) dz = 0;
    pos_z = pos_z + (dt*dt/2.0)*prev_acc_z + dt*vel_Z +(k1+k2*dt/2.0)*dt*dz;
    vel_Z = vel_Z + dt*prev_acc_z + k2*dt*dz;

    /*Serial.print(roll*180.0/PI);
    Serial.print(" ");
    Serial.print(pitch*180.0/PI);
    Serial.print(" ");
    Serial.println(pos_z);*/
    
    
    double ma = throttle;
    double mb = throttle;
    double mc = throttle;
    double md = throttle;

    double roll_rate_setpoint = kr*(roll_setpoint - roll*180.0/PI);
    double pitch_rate_setpoint = kr*(pitch_setpoint - pitch*180.0/PI);
    double yaw_rate_setpoint = 0;//kr*(yaw_setpoint - orientation.get_yaw()*180.0/PI);
    double vel_z_setpoint = ka*(alt_setpoint - pos_z);
    vel_z_setpoint = constrain(vel_z_setpoint, -2, 2);
    
    if(throttle >= 1100)
    {
      // Roll PID
      double roll_error = roll_rate_setpoint - roll_rate;
      double roll_diff = (roll_error - roll_prev_error)/dt;
      roll_integral += roll_error*dt;
      roll_integral = constrain(roll_integral, -100, 100);
      roll_prev_error = roll_error;
      double roll_pid = kp_roll*roll_error + ki_roll*roll_integral + kd_roll*roll_diff;

      // Pitch PID
      double pitch_error = pitch_rate_setpoint - pitch_rate;
      double pitch_diff = (pitch_error - pitch_prev_error)/dt;
      pitch_integral += pitch_error*dt;
      pitch_integral = constrain(roll_integral, -100, 100);
      pitch_prev_error = pitch_error;
      double pitch_pid = kp_pitch*pitch_error + ki_pitch*pitch_integral + kd_pitch*pitch_diff;

      // Yaw PID
      double yaw_error = yaw_rate_setpoint - yaw_rate;
      double yaw_diff = (yaw_error - yaw_prev_error)/dt;
      yaw_integral += yaw_error*dt;
      yaw_prev_error = yaw_error;
      double yaw_pid = kp_yaw*yaw_error + ki_yaw*yaw_integral + kd_yaw*yaw_diff;

      // Calculate PID if alt_hold is activated
      if(alt_hold)
      {
        double vel_z_error = vel_z_setpoint - vel_Z;
        vel_z_integral += vel_z_error*dt;
        vel_z_integral = constrain(vel_z_integral, -50, 50);
        vel_z_pid = kp_vel_z*vel_z_error + ki_vel_z*vel_z_integral - kd_vel_z*lpf_acc_z;
        //Serial.println(vel_Z);
      }

      ma = constrain(throttle + vel_z_pid - roll_pid/4.0 - pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      mb = constrain(throttle + vel_z_pid - roll_pid/4.0 + pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);
      mc = constrain(throttle + vel_z_pid + roll_pid/4.0 + pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      md = constrain(throttle + vel_z_pid + roll_pid/4.0 - pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);

      /*ma = constrain(throttle - roll_pid/4.0 - pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      mb = constrain(throttle - roll_pid/4.0 + pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);
      mc = constrain(throttle + roll_pid/4.0 + pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      md = constrain(throttle + roll_pid/4.0 - pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);*/
    }

    if(!eStop)
    {
      //Serial2.printf("%d,%.3f,%.2f,%.2f,%.2f\n", serial_counter, dt, roll*180/PI, kr*(roll_setpoint - roll*180.0/PI), roll_rate);
      Serial2.printf("%d,%.3f,%.2f,%.2f,%.2f, %.2f\n", serial_counter, dt, acc_x, roll*180.0/PI, acc_y, pitch*180/PI);
      serial_counter = (serial_counter+1)%10;
      ledcWrite(ledChannelA, MS_TO_PWM(ma));
      ledcWrite(ledChannelB, MS_TO_PWM(mb));
      ledcWrite(ledChannelC, MS_TO_PWM(mc));
      ledcWrite(ledChannelD, MS_TO_PWM(md));
      ledcWrite(ledChannelS1X, MS_TO_PWM(1500));
      ledcWrite(ledChannelS1Y, MS_TO_PWM(1500));
      ledcWrite(ledChannelS2X, MS_TO_PWM(1500));
      ledcWrite(ledChannelS2Y, MS_TO_PWM(1500));
      digitalWrite(LASER1, HIGH);
      digitalWrite(LASER2, HIGH);
    }
    
    update_orientation = 0;
  }

  if(update_altitude)
  {
    
    
    unsigned long current_time = millis();
    double dt = (current_time - altitude_prev_time)/1000.0;
    
    if(dt*1000.0 > 9.2)
    {
      altitude_prev_time = millis();
      tp_counter++;
      
      if(tp_counter == 20)
      {
        raw_temp = ms5611.readRawTemperature();
        ms5611.requestPressure();
        tp_counter = 0;
      }
      else if(tp_counter == 19)
      {
        unfiltered_pressure = ms5611.readPressure(raw_temp);
        ms5611.requestTemperature();
      }
      else
      {
        unfiltered_pressure = ms5611.readPressure(raw_temp);
        ms5611.requestPressure();
      }
  
      pres_av.sum -= pres_av.memory[pres_av.index];
      pres_av.memory[pres_av.index] = ms5611.getAltitude(unfiltered_pressure) - initial_alt;
      pres_av.sum += pres_av.memory[pres_av.index];
      pres_av.index++;
      if(pres_av.index == pres_av.samples) pres_av.index = 0;
      prev_pressure = pressure;
      pressure = pres_av.sum/pres_av.samples;
      
      update_altitude = 0;
    }
  }
}
