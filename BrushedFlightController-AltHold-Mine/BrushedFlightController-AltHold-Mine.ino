#include "BMX055.h"
#include "MS5611.h"
//#include "SensorFusion.h"
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "Estimates.h"

#define EEPROM_SIZE sizeof(float)*3

// BMX055 IMU addresses
#define AM_DEV 0x19//0x18
#define G_DEV 0x69//0x68
#define MAG_DEV 0x13////0x10

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
#define FREQ 32000 // 70000 was working well
#define RESOLUTION 10
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

// Constants for ESC control

#define MAX_PWM (1<<RESOLUTION)-1
//#define DUTY_TO_PWM(x) ((float)x)*((float)MAX_PWM)/100.0

// PID sampling
#define PID_SAMPLING 2800

// Class objects for data acquisition and sensor fusion
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
MS5611 altimeter = MS5611();
//SensorFusion orientation;

// RF input message structure
typedef struct received_message {
  uint8_t hr_stick;
  uint8_t vr_stick;
  uint8_t hl_stick;
  uint8_t vl_stick;
  uint8_t start;
  uint8_t select;
  uint8_t triangle;
  uint8_t circle;
  uint8_t cross;
  uint8_t square;
  uint8_t l_stick_button;
  uint8_t r_stick_button;
  uint8_t l_back_button;
  uint8_t r_back_button;
} received_message;

received_message controller_data;

// RF output message structure
typedef struct sent_message {
  float loop_time;
  float process_time;
  /*
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  */
  /*
  float pitch;
  float roll;
  float acc_z;
  */
  int16_t roll_rate_setpoint;
  int16_t pitch_rate_setpoint;
  int16_t yaw_rate_setpoint;
  int32_t altitude_setpoint;
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
  float altitude;
} sent_message;

sent_message drone_data;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t update_telemetry;

// RF interface variables
unsigned long prev_rf_time = 0;
uint8_t prev_stop = 0;
uint8_t prev_mode = 0;
uint8_t eStop = 1;
uint8_t altHold = 0;
uint8_t altHoldStart = 0;
int16_t rcData[4];
int16_t rcCommand[4];
int16_t initalThrottle = 0;

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
};

// Accelerometer calibration variables
uint8_t calibrate_acc = 0;
float acc_calibration[3] = {0,0,0};

// Orientation measurements
float roll_rate = 0;
float pitch_rate = 0;
float yaw_rate = 0;
float roll = 0;
float pitch = 0;

// Vertical acceleration
float acczSmooth = 0;
int32_t BaroPID = 0;
int32_t errorVelocityI = 0;
int32_t EstAlt = 0;
int32_t AltitudeSetpoint = 0;

// Baro calibration
uint8_t calibrate_alt;

// PID calculation variables
unsigned long prev_pid_time = 0;
int16_t axisPID[3];
float newAxisPID[3];
int16_t angleCorrection[2];

// Motor values
int16_t motor[4];

// Thread handles for tasks information display
TaskHandle_t rf_handle = NULL;

// RF reception callback
uint8_t msg_flag;
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&controller_data, incomingData, sizeof(controller_data));
  msg_flag = 1;
}

// PID timer ISR
hw_timer_t * pid_timer = NULL;
volatile uint8_t update_pid;
void IRAM_ATTR pid_isr() {
  update_pid = 1;
}

void rfLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {

    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_rf_time;
    if(dt > 500)
    {
      eStop = 1;
    }
    
    if(msg_flag)
    {
      prev_rf_time = current_time;

      // Joystick reading and mapping to correct ranges
      rcData[THROTTLE] = map(controller_data.vl_stick, 0, 255, 1000, 2000); // vertical left
      rcData[ROLL] = map(controller_data.hr_stick, 0, 255, -500, 500); // horizontal right
      rcData[PITCH] = map(controller_data.vr_stick, 0, 255, -500, 500); // vertical right
      rcData[YAW] = map(controller_data.hl_stick, 0, 255,-500, 500); // horizontal left

      // Exponential and ajusted scales
      rcCommand[THROTTLE] = map(constrain(rcData[THROTTLE], 1100, 2000), 1100, 2000, 1000, 2000);

      float x = ((float)abs(rcData[ROLL]))/128.0;
      rcCommand[ROLL] = ((1526.0 + 65.0*(x*x-15.0))*90.0*x)/1192.0;
      if(rcData[ROLL] < 0) rcCommand[ROLL] = -rcCommand[ROLL]; 

      x = ((float)abs(rcData[PITCH]))/128.0;
      rcCommand[PITCH] = ((1526.0 + 65.0*(x*x-15.0))*90.0*x)/1192.0;
      if(rcData[PITCH] < 0) rcCommand[PITCH] = -rcCommand[PITCH];
      
      rcCommand[YAW] = rcData[YAW];

      // Emergency stop simulated as a switch
      uint8_t temp_stop = controller_data.r_back_button;
      if(temp_stop == 1 && prev_stop == 0)
      { 
        if(eStop == 1 && rcData[THROTTLE] == 1000)
        {
          eStop = 0;
        }
        else
        {
          eStop = 1;
        }
        //eStop = !eStop;
      }
      prev_stop = temp_stop;

      // AltHold switch
      uint8_t temp_mode = controller_data.l_back_button;
      if(temp_mode == 1 && prev_mode == 0)
      {
        altHold = !altHold;
        if(altHold)
        {
          altHoldStart = 1;
        }
      }
      prev_mode = temp_mode;

      if(rcData[THROTTLE] > 1550)
      {
        digitalWrite(LED_PIN, HIGH);
      }
      else
      {
        digitalWrite(LED_PIN, LOW);
      }
      msg_flag = 0;
    }

    if(update_telemetry)
    {
      esp_now_send(broadcastAddress, (uint8_t *) &drone_data, sizeof(drone_data));
      update_telemetry = 0;
    }
    
    vTaskDelay(4);
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

  EEPROM.begin(EEPROM_SIZE);

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
  digitalWrite(LED_PIN, HIGH);

  delay(1000);

  // IMU intialization and calibration
  imu.acc_init();
  imu.gyr_init();

  //calibrate_acc = 1;

  // Altimeter initialization and calibration
  altimeter.begin(MS5611_ULTRA_HIGH_RES);
  altimeter.requestTemperature(); // First request is for pressure

  calibrate_alt = 1;
  
  int address = 0;
  EEPROM.get(address, acc_calibration);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Callback for message recepion
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  //Serial.println(ESP.getFreeHeap());

  // Orientation PID timer setup running at 250 Hz
  pid_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pid_timer, &pid_isr, true);
  timerAlarmWrite(pid_timer, PID_SAMPLING, true);
  
  // Blyn application running on core 0
  xTaskCreatePinnedToCore(
    rfLoop,      /* Function to implement the task */
    "RF core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &rf_handle,           /* Task handle. */
    0);             /* Core where the task should run */

  digitalWrite(LED_PIN, LOW);
  
  // Start pid timer
  timerAlarmEnable(pid_timer);
  
  Serial.println("Intial setup finished");
  //total heap = 177360

  prev_pid_time = micros();
  prev_rf_time = millis();
}

void loop(void)
{
  static uint8_t taskOrder=0;

  uint8_t axis;
  int16_t rc;
  int16_t error;
  int16_t delta;
  static int32_t errorGyroI_YAW;
  static int16_t errorGyroI[2] = {0,0};
  static float newErrorGyroI[2] = {0,0};
  static int16_t delta1[2],delta2[2];
  static float newDelta1[2],newDelta2[2];
  static int16_t lastGyro[2] = {0,0};
  static float newLastGyro[2] = {0,0};

  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;

  uint8_t kp = 12;//5;//12;
  uint8_t ki = 30;//17;//30;
  uint8_t kd = 23;//52;//23;

  float new_kp = 12;//5;//12;
  float new_ki = 30;//17;//30;
  float new_kd = 23;//52;//23;

  uint8_t level_kp = 45;//90
  uint8_t level_ki = 10;
  uint8_t level_kd = 100;

  uint8_t yaw_kp = 68;
  uint8_t yaw_ki = 45;
  uint8_t yaw_kd = 0;
  
  if(update_pid)
  {
    // Simple loop time verification
    unsigned long begining_time = micros();
    
    unsigned long current_time = micros();
    float dt = current_time - prev_pid_time;
    //Serial.println(dt, 0);
    prev_pid_time = current_time;
    
    
    if(rcData[THROTTLE] <= 1100)
    {
      errorGyroI[ROLL] = 0;
      errorGyroI[PITCH] = 0;

      newErrorGyroI[ROLL] = 0;
      newErrorGyroI[PITCH] = 0;
      
      errorGyroI_YAW = 0;
    }

    switch(taskOrder)
    {
      case 0:
        taskOrder++;
        if(get_baro_data() != 0)break;
      case 1:
        taskOrder=0;
        altitude_estimation();
        break;
    }

    if(altHoldStart)
    {
      altHoldStart = 0;
      errorVelocityI = 0;
      BaroPID = 0;
      initalThrottle  = rcCommand[THROTTLE];
      
      AltitudeSetpoint = EstAlt;
      //Serial.println(initalThrottle);
    }

    if(altHold)
    {
      //rcCommand[THROTTLE] = 1590;// Discharged battery
      //rcCommand[THROTTLE] = 1540;// Full battery battery
      rcCommand[THROTTLE] = 1565 + BaroPID;
      //rcCommand[THROTTLE] = initalThrottle + BaroPID;
    }
    
    // Get IMU data
    get_gyr_compensated_data();
    get_acc_compensated_data();

    // Estimate euler angles, just used to get world frame acceleration
    attitude_estimation(dt);
    // Get world frame z acceleration for AltHold
    acceleration_estimation(dt);
    
    int16_t angle[2];
    angle[ROLL] = roll*180.0*10.0/PI;
    angle[PITCH] = pitch*180.0*10.0/PI;

    roll_rate = roll_rate*0.7 + ((int16_t)(imu.gyroscope.x*imu.gyroscope.res*16.4)>>2)*0.3;
    pitch_rate = pitch_rate*0.7 + ((int16_t)(imu.gyroscope.y*imu.gyroscope.res*16.4)>>2)*0.3;
    yaw_rate = yaw_rate*0.7 + ((int16_t)(imu.gyroscope.z*imu.gyroscope.res*16.4)>>2)*0.3;

    //****PENGIND STEPS****//
    /*
    1. Integrgate Baro Mode button from controller
    //Integrate Baro calibration on arm -> Not necessary
    2. Reset PID stuff on baro mode activation and save throttle value
    3. Add PID calculation to loop
    */

    int16_t gyroData[3];
    gyroData[ROLL] = (int16_t)roll_rate;
    gyroData[PITCH] = (int16_t)pitch_rate;
    gyroData[YAW] = (int16_t)yaw_rate;

    float newGyroData[3];
    newGyroData[ROLL] = roll_rate;
    newGyroData[PITCH] = pitch_rate;
    newGyroData[YAW] = yaw_rate;

    // PITCH & ROLL
    for(axis=0;axis<2;axis++) {
      float sp = (float)rcCommand[axis]*0.3;
      float new_error = sp - newGyroData[axis];
      newErrorGyroI[axis] = constrain(newErrorGyroI[axis]+new_error,-16000,+16000);
      if (abs(newGyroData[axis])>640) newErrorGyroI[axis] = 0;

      float newITerm = (newErrorGyroI[axis]/128.0)*new_ki/64.0;

      float newPTerm = sp*new_kp/64.0;
      newPTerm -= (newGyroData[axis]*new_kp)/64.0;

      float newDelta = newGyroData[axis] - newLastGyro[axis];
      newLastGyro[axis] = newGyroData[axis];

      float newDTerm = newDelta1[axis] + newDelta2[axis] + newDelta;
      newDelta2[axis]   = newDelta1[axis];
      newDelta1[axis]   = newDelta;

      newDTerm = (newDTerm*new_kd)/32.0;

      newAxisPID[axis]  = newPTerm + newITerm - newDTerm;

      axisPID[axis] = newAxisPID[axis];
    }
    
    //YAW
    #define GYRO_P_MAX 300
    #define GYRO_I_MAX 250
  
    rc = rcCommand[YAW] * 30  >> 5;

    error = rc - gyroData[YAW];
    errorGyroI_YAW  += error*yaw_ki;
    errorGyroI_YAW  = constrain(errorGyroI_YAW, 2-((int32_t)1<<28), -2+((int32_t)1<<28));
    if (abs(rc) > 50) errorGyroI_YAW = 0;

    PTerm = error*kp>>6;
    ITerm = constrain((int16_t)(errorGyroI_YAW>>13),-GYRO_I_MAX,+GYRO_I_MAX);

    axisPID[YAW] =  PTerm + ITerm;

    // Mix table
    motor[0] = rcCommand[THROTTLE] - axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW];
    motor[1] = rcCommand[THROTTLE] + axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW];
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW];
    motor[3] = rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW];

    int16_t maxMotor;
    uint8_t i;

    maxMotor=motor[0];
    for(i=1; i< 4; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< 4; i++) {
      if (maxMotor > 2000) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - 2000;
      motor[i] = constrain(motor[i], 1000, 2000);
      if ((rcData[THROTTLE] < 1100))
        motor[i] = 1000;
      if (eStop)
        motor[i] = 1000;
    }

    for(i=0; i< 4; i++) {
      motor[i] = map(motor[i], 1000, 2000, 0, MAX_PWM);
    }

    ledcWrite(ledChannelA, motor[0]);
    ledcWrite(ledChannelB, motor[1]);
    ledcWrite(ledChannelC, motor[2]);
    ledcWrite(ledChannelD, motor[3]);

    // Log some values
    //Serial.print(uxTaskGetStackHighWaterMark(blynk_handle));
    //Serial.print("\t");
    //Serial.print(uxTaskGetStackHighWaterMark(imu_handle));

    //Serial.print(angle[ROLL]/10.0);
    //Serial.print(" ");
    //Serial.println(angle[PITCH]/10.0);

    /*Serial.print(gyroData[ROLL]);
    Serial.print(" ");
    Serial.println(gyroData[PITCH]);*/

    //Serial.print(roll_rate);
    //Serial.print(" ");
    //Serial.println(pitch_rate);
    
    update_pid = 0;

    float elapsed_time = micros() - begining_time;

    // Telemetry update
    drone_data.loop_time = dt;
    drone_data.process_time = elapsed_time;
    /*
    drone_data.acc_x = imu.accelerometer.x*imu.accelerometer.res;
    drone_data.acc_y = imu.accelerometer.y*imu.accelerometer.res;
    drone_data.acc_z = imu.accelerometer.z*imu.accelerometer.res;

    drone_data.gyr_x = imu.gyroscope.x*imu.gyroscope.res;
    drone_data.gyr_y = imu.gyroscope.y*imu.gyroscope.res;
    drone_data.gyr_z = imu.gyroscope.z*imu.gyroscope.res;
    */
    /*
    drone_data.roll = roll;
    drone_data.pitch = pitch;
    drone_data.acc_z = acczSmooth;
    */

    drone_data.roll_rate_setpoint = rcCommand[ROLL]*0.3;
    drone_data.pitch_rate_setpoint = rcCommand[PITCH]*0.3;
    drone_data.yaw_rate_setpoint = rcCommand[YAW]*30>>5;
    drone_data.altitude_setpoint = AltitudeSetpoint;
    drone_data.roll_rate = roll_rate;
    drone_data.pitch_rate = pitch_rate;
    drone_data.yaw_rate = yaw_rate;
    drone_data.altitude = EstAlt;
    
    // Set this to 0 if not needed to send data
    update_telemetry = 1;
    
    //Serial.println(elapsed_time, 0);
  }
}
