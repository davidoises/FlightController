#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10
#define SERVO1_X 13
#define SERVO1_Y 12
#define SERVO2_X 26
#define SERVO2_Y 25
#define VBAT_ADC 37
#define ADCA 37
#define ADCB 38
#define ADCC 34
#define ADCD 35
#define LASER1 0
#define LASER2 2
#define INTERRUPT_PIN 19

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

// UI Control variables
float eStop = 1;
float throttle = 1000.0;
float roll = 1500.0;
float pitch = 1500.0;
float yaw = 1500.0;
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

#define ORIENTATION_SAMPLING 0.004f

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
int16_t gyr[3]; // [roll, pitch, yaw]

// PID calculation variables
unsigned long prev_time = 0;
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

hw_timer_t * orientation_timer = NULL;
volatile uint8_t update_orientation;
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
}


void blynkLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {
    //Serial.println("hola");
    Blynk.run();
    vTaskDelay(50);
  }
}
void mpu_loop(void *pvParameters )
{
  while(true)
  {
    // if programming failed, don't try to do anything
    //if (!dmpReady) return;
    while(!dmpReady)
    {
      vTaskDelay(50);
    }
  
    // wait for MPU interrupt or extra packet(s) available
    //if (!mpuInterrupt && fifoCount < packetSize) return;
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

      // Get Gyroscope values
      mpu.dmpGetGyro(gyr, fifoBuffer);
      
      // Get yaw pitch roll values
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 
      // get accelerometer values (rotated to world frame)
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
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

  // MPU6050 init
  Serial.println(F("Initializing MPU6050 devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
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
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);
  
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
  timerAlarmEnable(orientation_timer);
  Serial.println("Intial setup finished");
  //total heap = 177360

  prev_time = millis();
}

void loop(void)
{
  // Dont let anything run under emergency stop
  if(eStop)
  {
    ledcWrite(ledChannelA, MS_TO_PWM(1000));
    ledcWrite(ledChannelB, MS_TO_PWM(1000));
    ledcWrite(ledChannelC, MS_TO_PWM(1000));
    ledcWrite(ledChannelD, MS_TO_PWM(1000));
  }
  
  if(update_orientation)
  {
    // Calculate elapse time (shoule be exactly 0.004s)
    unsigned long current_time = millis();
    float dt = (current_time - prev_time)/1000.0;
    prev_time = current_time;

    // Variables for ESC pwm values
    float ma = throttle;
    float mb = throttle;
    float mc = throttle;
    float md = throttle;

    // P controller for angular position
    float roll_rate_setpoint = kc_roll*(roll - ypr[2]);
    float pitch_rate_setpoint = kc_pitch*(pitch - ypr[1]);
    
    if(throttle >= 1100)
    {
      // PD controller for angular velocity - roll
      float roll_rate_error = roll_rate_setpoint - gyr[0];
      float roll_rate_error_diff = (roll_rate_error - prev_roll_rate_error)/dt;
      prev_roll_rate_error = roll_rate_error;
      float roll_pid = kp_roll_rate*roll_rate_error + kd_roll_rate*roll_rate_error_diff;

      // PD controller for angular velocity - pitch
      float pitch_rate_error = pitch_rate_setpoint - gyr[1];
      float pitch_rate_error_diff = (pitch_rate_error - prev_pitch_rate_error)/dt;
      prev_pitch_rate_error = pitch_rate_error;
      float pitch_pid = kp_pitch_rate*pitch_rate_error + kd_pitch_rate*pitch_rate_error_diff;

      // PD controller for angular velocity - yaw
      float yaw_rate_error = yaw - gyr[2];
      float yaw_rate_error_diff = (yaw_rate_error - prev_yaw_rate_error)/dt;
      prev_yaw_rate_error = yaw_rate_error;
      float yaw_pid = kp_yaw_rate*yaw_rate_error + kd_yaw_rate*yaw_rate_error_diff;

      // Control law signals mixer according to quad X configuration
      ma = throttle - roll_pid/4.0 - pitch_pid/4.0 + yaw_pid/4.0;
      mb = throttle - roll_pid/4.0 + pitch_pid/4.0 - yaw_pid/4.0;
      mc = throttle + roll_pid/4.0 + pitch_pid/4.0 + yaw_pid/4.0;
      md = throttle + roll_pid/4.0 - pitch_pid/4.0 - yaw_pid/4.0;
    }
    else
    {
      ma = 1000.0;
      mb = 1000.0;
      mc = 1000.0;
      md = 1000.0;
    }

    // If not under emergency stop, apply pwm
    if(!eStop)
    {
      ledcWrite(ledChannelA, MS_TO_PWM(ma));
      ledcWrite(ledChannelB, MS_TO_PWM(mb));
      ledcWrite(ledChannelC, MS_TO_PWM(mc));
      ledcWrite(ledChannelD, MS_TO_PWM(md));
    }

    // Log some values
    Serial.print(uxTaskGetStackHighWaterMark(blynk_handle));
    Serial.print("\t");
    Serial.print(uxTaskGetStackHighWaterMark(imu_handle));
    Serial.print("\t");
    Serial.print(dt*1000.0);
    Serial.print("\t");
    Serial.print(throttle);
    Serial.print("\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);

    update_orientation = 0;
  }
}
