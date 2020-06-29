#include "BMX055.h"
#include "SensorFusion.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 12
#define VBAT_ADC 27
#define IMU_INTERRUPT 25
#define RF_INTERRUPT 15
#define LED_PIN 0

// setting PWM properties
#define FREQ 30000
#define RESOLUTION 10
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

// Constants for ESC control

#define MAX_PWM (1<<RESOLUTION)-1
//#define DUTY_TO_PWM(x) ((float)x)*((float)MAX_PWM)/100.0

// PID sampling
#define PID_SAMPLING 0.004f

// Class objects for data acquisition and sensor fusion
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
SensorFusion orientation;
RF24 radio(4, 5, 18, 19, 23); // CE, CSN, SCK, MISO, MOSI

// RF Message packet variables
struct RF24Data {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
  byte switches;
};
const uint64_t pipeIn = 0xE8E8F0F0E1LL;  //Remember, SAME AS TRANSMITTER CODE

// Orientation measurements
unsigned long prev_imu_time = 0;
float roll = 0;
float pitch = 0;
float yaw = 0;
float roll_rate = 0;
float pitch_rate = 0;
float yaw_rate = 0;
float prev_roll_rate = 0;
float prev_pitch_rate = 0;
float prev_yaw_rate = 0;
float roll_rate_d = 0;
float pitch_rate_d = 0;
float yaw_rate_d = 0;

// RF interface variables
RF24Data remote_data;
unsigned long prev_rf_time = 0;
uint8_t eStop = 1;
float throttle = 0;
float roll_setpoint = 0;
float pitch_setpoint = 0;
float yaw_setpoint = 0;

// PID gains
float kc_roll = 5.0;
float kc_pitch = 5.0;

float kp_roll_rate = 1.5;
float kd_roll_rate = 0.2;

float kp_pitch_rate = 1.5;
float kd_pitch_rate = 0.2;

float kp_yaw_rate = 0.5;
float kd_yaw_rate = 0;

// PID calculation variables
unsigned long prev_pid_time = 0;
float prev_roll_rate_error = 0;
float prev_pitch_rate_error = 0;
float prev_yaw_rate_error = 0;

// Thread handles for tasks information display
TaskHandle_t imu_handle = NULL;
TaskHandle_t rf_handle = NULL;

// RF reception ISR
volatile uint8_t msg_flag;
void msg_flag_isr()
{
  msg_flag = 1; 
}

// PID timer ISR
hw_timer_t * pid_timer = NULL;
volatile uint8_t update_pid;
void IRAM_ATTR pid_isr() {
  update_pid = 1;
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rfLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {

    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_rf_time;
    if(dt > 1000)
    {
      eStop = 1;
    }

    if(msg_flag)
    {
      prev_rf_time = current_time;

      radio.read(&remote_data, sizeof(RF24Data));

      // Joystick reading and mapping to correct ranges
      throttle = map_float(remote_data.throttle, 0, 255, 0, 850); // vertical left
      roll_setpoint = map_float(remote_data.roll, 0, 255, -10, 10); // horizontal right
      pitch_setpoint = map_float(remote_data.pitch, 0, 255, -10, 10); // vertical right
      //yaw_setpoint = map_float(remote_data.yaw, 0, 255, -1, 1); // horizontal left

      eStop = !remote_data.AUX1;

      if(throttle > 550)
      {
        digitalWrite(LED_PIN, HIGH);
      }
      else
      {
        digitalWrite(LED_PIN, LOW);
      }

      /*
      Serial.print(throttle);
      Serial.print(" ");
      Serial.print(roll_setpoint);
      Serial.print(" ");
      Serial.println(pitch_setpoint);
      */
      
      msg_flag = false;
    }
    
    vTaskDelay(1);
  }
}
void mpu_loop(void *pvParameters )
{
  while(true)
  {
    /*
    unsigned long current_time = millis();
    float dt = (current_time - prev_imu_time)/1000.0;
    prev_imu_time = current_time;

    imu.get_gyr_data();
    imu.get_acc_data();

    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
    
    roll = orientation.get_roll()*180.0/PI;
    pitch = orientation.get_pitch()*180.0/PI;
    //yaw = orientation.get_yaw()*180.0/PI;

    //Serial.print(imu.accelerometer.x);
    //Serial.print(" ");
    //Serial.println(pitch);
    
    roll_rate = roll_rate*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
    pitch_rate = pitch_rate*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
    yaw_rate = yaw_rate*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;

    Serial.print(dt);
    Serial.print(" ");
    Serial.print(imu.gyroscope.x*imu.gyroscope.res);
    Serial.print(" ");
    Serial.println(imu.gyroscope.y*imu.gyroscope.res);
    //Serial.print(" ");
    //Serial.println(imu.gyroscope.z*imu.gyroscope.res);

    roll_rate_d = (roll_rate - prev_roll_rate)/dt;
    pitch_rate_d = (pitch_rate - prev_pitch_rate)/dt;
    yaw_rate_d = (yaw_rate - prev_yaw_rate)/dt;*/

    vTaskDelay(8);

  }
}

void setup(void)
{
  // Start UART Bus
  //Serial.begin(115200);
  Serial.begin(1000000);
  Serial.println("");
  Serial.println("Starting system");
  
  // Start SPI Bus
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.

  // Initialize PWM channels
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA); // negative torque
  ledcWrite(ledChannelA, 0);

  ledcSetup(ledChannelB, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, ledChannelB); // negative torque
  ledcWrite(ledChannelB, 0);

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC); // Positive toruqe
  ledcWrite(ledChannelC, 0);

  ledcSetup(ledChannelD, FREQ, RESOLUTION);
  ledcAttachPin(PWMD, ledChannelD); // Positive toruqe
  ledcWrite(ledChannelD, 0);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // IMU intialization and calibration
  imu.acc_init();
  imu.gyr_init();
  /*for(int i = 0; i < 200; i++)
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
    //initial_acc += az*imu.accelerometer.res;
    delay(4);
  }
  initial_acc_roll /= 100;
  initial_acc_pitch /= 100;
  orientation.init(initial_acc_roll, initial_acc_pitch, MAG_TARGET);
  Serial.print(initial_acc_roll, 6);
  Serial.print(" ");
  Serial.println(initial_acc_pitch, 6);
  delay(5000);*/
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
    initial_acc_roll += atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) - (-0.20243775);
    initial_acc_pitch += atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2))) - (0.146825);
    //initial_acc += az*imu.accelerometer.res;
    delay(4);
  }
  initial_acc_roll /= 100;
  initial_acc_pitch /= 100;
  orientation.init(initial_acc_roll, initial_acc_pitch, MAG_TARGET);
  //orientation.init(-0.20243775, 0.146825, MAG_TARGET);

  // Start Rf interface
  radio.begin();
  radio.maskIRQ(1,1,0);
  //radio.setAutoAck(false);
  radio.openReadingPipe(0, pipeIn);
  radio.startListening();
  
  pinMode(RF_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(RF_INTERRUPT, msg_flag_isr, FALLING);
  
  //Serial.println(ESP.getFreeHeap());

  // Orientation PID timer setup running at 250 Hz
  pid_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pid_timer, &pid_isr, true);
  timerAlarmWrite(pid_timer, PID_SAMPLING*1000000.0, true);
  
  // Blyn application running on core 0
  xTaskCreatePinnedToCore(
    rfLoop,      /* Function to implement the task */
    "RF core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &rf_handle,           /* Task handle. */
    0);             /* Core where the task should run */

  // IMU measurements running on core 0
  xTaskCreatePinnedToCore(
    mpu_loop,      /* Function to implement the task */
    "IMU core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    2,              /* Priority of the task */
    &imu_handle,           /* Task handle. */
    0);             /* Core where the task should run */

  // Start pid timer
  timerAlarmEnable(pid_timer);
  
  Serial.println("Intial setup finished");
  //total heap = 177360

  prev_imu_time = millis();
  prev_pid_time = millis();
  prev_rf_time = millis();
}

void loop(void)
{
  // Dont let anything run under emergency stop
  if(eStop)
  {
    ledcWrite(ledChannelA, 0);
    ledcWrite(ledChannelB, 0);
    ledcWrite(ledChannelC, 0);
    ledcWrite(ledChannelD, 0);
  }
  
  if(update_pid)
  {
    // Calculate elapse time (shoule be exactly 0.004s)
    unsigned long current_time = millis();
    float dt = (current_time - prev_pid_time)/1000.0;
    prev_pid_time = current_time;

    
    imu.get_gyr_data();
    imu.get_acc_data();

    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
    
    roll = orientation.get_roll()*180.0/PI;
    pitch = orientation.get_pitch()*180.0/PI;
    //yaw = orientation.get_yaw()*180.0/PI;

    roll_rate = roll_rate*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
    pitch_rate = pitch_rate*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
    yaw_rate = yaw_rate*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;

    /*roll_rate_d = (roll_rate - prev_roll_rate)/dt;
    pitch_rate_d = (pitch_rate - prev_pitch_rate)/dt;
    yaw_rate_d = (yaw_rate - prev_yaw_rate)/dt;*/
    
    // Variables for ESC pwm values
    float ma = throttle;
    float mb = throttle;
    float mc = throttle;
    float md = throttle;

    // P controller for angular position
    float roll_rate_setpoint = kc_roll*(roll_setpoint - roll);
    float pitch_rate_setpoint = kc_pitch*(pitch_setpoint - pitch);
    
    if(throttle >= 0.1*MAX_PWM)
    {
      // PD controller for angular velocity - roll
      float roll_rate_error = roll_rate_setpoint - roll_rate;
      float roll_rate_error_diff = (roll_rate_error - prev_roll_rate_error)/dt;
      prev_roll_rate_error = roll_rate_error;
      float roll_pid = kp_roll_rate*roll_rate_error + kd_roll_rate*roll_rate_error_diff;

      // PD controller for angular velocity - pitch
      float pitch_rate_error = pitch_rate_setpoint - pitch_rate;
      float pitch_rate_error_diff = (pitch_rate_error - prev_pitch_rate_error)/dt;
      prev_pitch_rate_error = pitch_rate_error;
      float pitch_pid = kp_pitch_rate*pitch_rate_error + kd_pitch_rate*pitch_rate_error_diff;

      // PD controller for angular velocity - yaw
      float yaw_rate_error = yaw_setpoint - yaw_rate;
      float yaw_rate_error_diff = (yaw_rate_error - prev_yaw_rate_error)/dt;
      prev_yaw_rate_error = yaw_rate_error;
      float yaw_pid = kp_yaw_rate*yaw_rate_error + kd_yaw_rate*yaw_rate_error_diff;

      // Control law signals mixer according to quad X configuration
      ma = throttle - roll_pid/4.0 + pitch_pid/4.0 + yaw_pid/4.0;
      mb = throttle + roll_pid/4.0 + pitch_pid/4.0 - yaw_pid/4.0;
      mc = throttle + roll_pid/4.0 - pitch_pid/4.0 + yaw_pid/4.0;
      md = throttle - roll_pid/4.0 - pitch_pid/4.0 - yaw_pid/4.0;
    }
    else
    {
      ma = 0;
      mb = 0;
      mc = 0;
      md = 0;
    }

    // If not under emergency stop, apply pwm
    if(!eStop)
    {
      
      ma = 0;
      mb = 0;
      mc = 0;
      md = 0;
      
      ma = throttle;
      //mb = throttle;
      //mc = throttle;
      //md = throttle;
      ledcWrite(ledChannelA, ma);
      ledcWrite(ledChannelB, mb);
      ledcWrite(ledChannelC, mc);
      ledcWrite(ledChannelD, md);
    }

    // Log some values
    //Serial.print(uxTaskGetStackHighWaterMark(blynk_handle));
    //Serial.print("\t");
    //Serial.print(uxTaskGetStackHighWaterMark(imu_handle));
    //Serial.println(throttle);

    Serial.print(throttle/100, 0);
    Serial.print(" ");
    Serial.print(roll, 1);
    Serial.print(" ");
    Serial.println(pitch, 1);
    
    
    update_pid = 0;
  }
}
