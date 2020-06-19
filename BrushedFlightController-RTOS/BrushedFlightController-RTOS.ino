#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 12
#define VBAT_ADC 27
#define IMU_INTERRUPT 19// 25
#define RF_INTERRUPT 15

// setting PWM properties
#define FREQ 30000
#define RESOLUTION 8
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define DUTY_TO_PWM(x) ((float)x)*((float)MAX_PWM)/100.0

// PID sampling
#define PID_SAMPLING 0.004f

// UI Control variables
float eStop = 1;
float throttle = 0;
float roll_setpoint = 0;
float pitch_setpoint = 0;
float yaw_setpoint = 0;
float aux1 = 1000.0;
float ui_callback = 0;

float kc_roll = 0;
float kc_pitch = 0;

float kp_roll_rate = 0;//3
float kd_roll_rate = 0;//0.3

float kp_pitch_rate = 0;//3*1.08
float kd_pitch_rate = 0;//0.3

float kp_yaw_rate = 0.001;//1.0; //0.002
float kd_yaw_rate = 0;

#include "ui_conf.h"

MPU6050 mpu; //0x68 default I2C address

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float gyr[3]; // [yaw, pitch, roll]
float diff_gyr[3]; // [yaw, pitch, roll]
float prev_gyr[3]; // [yaw, pitch, roll]
unsigned long prev_imu_time = 0;

// PID calculation variables
unsigned long prev_pid_time = 0;
float prev_roll_rate_error = 0;
float prev_pitch_rate_error = 0;
float prev_yaw_rate_error = 0;

// Thread handles for tasks information display
TaskHandle_t imu_handle = NULL;
TaskHandle_t blynk_handle = NULL;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

hw_timer_t * pid_timer = NULL;
volatile uint8_t update_pid;
void IRAM_ATTR pid_isr() {
  update_pid = 1;
}


void blynkLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {
    Blynk.run();
    vTaskDelay(50);
  }
}
void mpu_loop(void *pvParameters )
{
  while(true)
  {
    // if programming failed, don't try to do anything
    while(!dmpReady)
    {
      vTaskDelay(50);
    }
  
    // wait for MPU interrupt or extra packet(s) available
    while(!mpuInterrupt && fifoCount < packetSize)
    {
      vTaskDelay(50);
    }
  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
  
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      unsigned long current_time = millis();
      float dt = (current_time - prev_imu_time)/1000.0;
      prev_imu_time = current_time;

      // Gyroscope values obtention
      int16_t temp[3];
      mpu.dmpGetGyro(temp, fifoBuffer);
      gyr[0] = (float)temp[2];
      gyr[1] = (float)temp[1];
      gyr[2] = (float)temp[0];
      
      // Get yaw pitch roll values
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 
      // Gyroscope derivated values
      diff_gyr[0] = (gyr[0] - prev_gyr[0])/dt;
      diff_gyr[1] = (gyr[1] - prev_gyr[1])/dt;
      diff_gyr[2] = (gyr[2] - prev_gyr[2])/dt;
      prev_gyr[0] = gyr[0];
      prev_gyr[1] = gyr[1];
      prev_gyr[2] = gyr[2];
    }
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
  ledcWrite(ledChannelA, DUTY_TO_PWM(0));

  ledcSetup(ledChannelB, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, ledChannelB); // negative torque
  ledcWrite(ledChannelB, DUTY_TO_PWM(0));

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC); // Positive toruqe
  ledcWrite(ledChannelC, DUTY_TO_PWM(0));

  ledcSetup(ledChannelD, FREQ, RESOLUTION);
  ledcAttachPin(PWMD, ledChannelD); // Positive toruqe
  ledcWrite(ledChannelD, DUTY_TO_PWM(0));

  // MPU6050 init
  Serial.println(F("Initializing MPU6050"));
  mpu.initialize();
  pinMode(IMU_INTERRUPT, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();

  // Gyro offset settings
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection"));
    attachInterrupt(IMU_INTERRUPT, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Start user interface while ESCs set up
  #if WIFI_GUI
  Blynk.begin(auth, ssid, pass);
  #else
  Blynk.begin(auth);
  #endif
  
  //Serial.println(ESP.getFreeHeap());

  // Orientation PID timer setup running at 250 Hz
  pid_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pid_timer, &pid_isr, true);
  timerAlarmWrite(pid_timer, PID_SAMPLING*1000000.0, true);
  
  // Blyn application running on core 0
  xTaskCreatePinnedToCore(
    blynkLoop,      /* Function to implement the task */
    "blynk core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &blynk_handle,           /* Task handle. */
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
}

void loop(void)
{
  // Dont let anything run under emergency stop
  if(eStop)
  {
    ledcWrite(ledChannelA, DUTY_TO_PWM(0));
    ledcWrite(ledChannelB, DUTY_TO_PWM(0));
    ledcWrite(ledChannelC, DUTY_TO_PWM(0));
    ledcWrite(ledChannelD, DUTY_TO_PWM(0));
  }
  
  if(update_pid)
  {
    // Calculate elapse time (shoule be exactly 0.004s)
    unsigned long current_time = millis();
    float dt = (current_time - prev_pid_time)/1000.0;
    prev_pid_time = current_time;

    float roll = ypr[2];
    float pitch = -ypr[1];
    float yaw = -ypr[0];

    //Serial.print(yaw * 180/M_PI); // yaw
    //Serial.print(",");
    //Serial.print(pitch * 180/M_PI); // pitch
    //Serial.print(",");
    //Serial.print(roll * 180/M_PI); // roll
    //Serial.print(",");
    
    //Serial.print(gyr[0]); // yaw
    //Serial.print(",");
    //Serial.print(gyr[1]); // pitch
    //Serial.print(",");
    //Serial.print(gyr[2]); // roll
    //Serial.print(",");
    
    //Serial.print(diff_gyr[0]); // yaw
    //Serial.print(",");
    //Serial.print(diff_gyr[1]); // pitch
    //Serial.print(",");
    //Serial.println(diff_gyr[2]); // roll
    
    
    // Variables for ESC pwm values
    float ma = throttle;
    float mb = throttle;
    float mc = throttle;
    float md = throttle;

    // P controller for angular position
    float roll_rate_setpoint = kc_roll*(roll_setpoint - roll);
    float pitch_rate_setpoint = kc_pitch*(pitch_setpoint - pitch);
    
    if(throttle >= 10)
    {
      // PD controller for angular velocity - roll
      float roll_rate_error = roll_rate_setpoint - gyr[2];
      float roll_rate_error_diff = (roll_rate_setpoint - prev_roll_rate_error)/dt - diff_gyr[2];
      prev_roll_rate_error = roll_rate_setpoint;
      float roll_pid = kp_roll_rate*roll_rate_error + kd_roll_rate*roll_rate_error_diff;

      // PD controller for angular velocity - pitch
      float pitch_rate_error = pitch_rate_setpoint - gyr[1];
      float pitch_rate_error_diff = (pitch_rate_setpoint - prev_pitch_rate_error)/dt - diff_gyr[1];
      prev_pitch_rate_error = pitch_rate_setpoint;
      float pitch_pid = kp_pitch_rate*pitch_rate_error + kd_pitch_rate*pitch_rate_error_diff;

      // PD controller for angular velocity - yaw
      float yaw_rate_error = yaw_setpoint - gyr[0];
      float yaw_rate_error_diff = (yaw_setpoint - prev_yaw_rate_error)/dt - diff_gyr[0];
      prev_yaw_rate_error = yaw_setpoint;
      float yaw_pid = kp_yaw_rate*yaw_rate_error + kd_yaw_rate*yaw_rate_error_diff;

      // Control law signals mixer according to quad X configuration
      ma = throttle - roll_pid/4.0 - pitch_pid/4.0 + yaw_pid/4.0;
      mb = throttle - roll_pid/4.0 + pitch_pid/4.0 - yaw_pid/4.0;
      mc = throttle + roll_pid/4.0 + pitch_pid/4.0 + yaw_pid/4.0;
      md = throttle + roll_pid/4.0 - pitch_pid/4.0 - yaw_pid/4.0;
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
      ledcWrite(ledChannelA, DUTY_TO_PWM(ma));
      ledcWrite(ledChannelB, DUTY_TO_PWM(mb));
      ledcWrite(ledChannelC, DUTY_TO_PWM(mc));
      ledcWrite(ledChannelD, DUTY_TO_PWM(md));
    }

    // Log some values
    //Serial.print(uxTaskGetStackHighWaterMark(blynk_handle));
    //Serial.print("\t");
    //Serial.print(uxTaskGetStackHighWaterMark(imu_handle));
    //Serial.print("\t");
    //Serial.print(dt*1000.0);
    //Serial.print("\t");
    //Serial.print(throttle);
    //Serial.print("\t");
    //Serial.print(ypr[0] * 180/M_PI);
    //Serial.print("\t");
    //Serial.print(ypr[1] * 180/M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[2] * 180/M_PI);

    update_pid = 0;
  }
}
