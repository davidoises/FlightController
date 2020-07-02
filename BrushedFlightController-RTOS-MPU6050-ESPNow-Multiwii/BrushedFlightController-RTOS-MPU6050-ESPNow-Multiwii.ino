#include "BMX055.h"
#include "SensorFusion.h"
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

#define EEPROM_SIZE sizeof(float)*2

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
SensorFusion orientation;

// RF message structure
typedef struct remote_message {
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
} remote_message;

remote_message controller_data;

// RF interface variables
unsigned long prev_rf_time = 0;
uint8_t prev_stop = 0;
uint8_t eStop = 1;
int16_t rcData[4];
int16_t rcCommand[4];

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
};

// Orientation measurements
float roll_rate = 0;
float pitch_rate = 0;
float yaw_rate = 0;

// PID calculation variables
unsigned long prev_pid_time = 0;
int16_t axisPID[3];
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
    
    vTaskDelay(20);
  }
}

void calibrateAcc()
{
  for(int i = 0; i < 200; i++)
  {
    imu.get_acc_data();
    delay(5);
  }
  float initial_acc_roll = 0;
  float initial_acc_pitch = 0;
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

  int address = 0;

  EEPROM.put(address, initial_acc_roll);
  //EEPROM.commit();
  address += sizeof(float);
  EEPROM.put(address, initial_acc_pitch);
  EEPROM.commit();

  orientation.set_acc_offsets(initial_acc_roll, initial_acc_pitch);

  /*
  Serial.print(initial_acc_roll, 6);
  Serial.print(" ");
  Serial.println(initial_acc_pitch, 6);
  delay(5000);
  */
}

void initalAngle()
{
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
    initial_acc_roll += atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) - orientation.roll_offset;
    initial_acc_pitch += atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2))) - orientation.pitch_offset;
    //initial_acc += az*imu.accelerometer.res;
    delay(4);
  }
  initial_acc_roll /= 100;
  initial_acc_pitch /= 100;
  Serial.print(initial_acc_roll*180.0/PI);
  Serial.print(" ");
  Serial.println(initial_acc_pitch*180.0/PI);
  orientation.init(initial_acc_roll, initial_acc_pitch, MAG_TARGET);
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

  //calibrateAcc();

  float roll_offset = 0;
  float pitch_offset = 0;

  int address = 0;
  EEPROM.get(address, roll_offset);
  address += sizeof(float);
  EEPROM.get(address, pitch_offset);
  
  orientation.set_acc_offsets(roll_offset, pitch_offset); // This should be read from an EEPROM

  initalAngle();
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Callback for message recepion
  esp_now_register_recv_cb(OnDataRecv);
  
  
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

  uint8_t axis;
  int16_t rc;
  int16_t error, errorAngle;
  int16_t delta;
  static int32_t errorGyroI_YAW;
  static int16_t errorGyroI[2] = {0,0};
  static int16_t errorAngleI[2] = {0,0};
  static int16_t delta1[2],delta2[2];
  static int16_t lastGyro[2] = {0,0};

  int16_t PTerm = 0,ITerm = 0,DTerm, PTermACC, ITermACC;

  uint8_t kp = 12;//33
  uint8_t ki = 30;
  uint8_t kd = 23;

  uint8_t level_kp = 45;//90
  uint8_t level_ki = 10;
  uint8_t level_kd = 100;

  uint8_t yaw_kp = 68;
  uint8_t yaw_ki = 45;
  uint8_t yaw_kd = 0;
  
  if(update_pid)
  {
    // Simple loop time verification
    
    /*
    unsigned long current_time = micros();
    float dt = current_time - prev_pid_time;
    Serial.println(dt, 0);
    prev_pid_time = current_time;
    */
    

    //unsigned long begining_time = micros();
    
    if(rcData[THROTTLE] <= 1100)
    {
      errorGyroI[ROLL] = 0;
      errorGyroI[PITCH] = 0;
      errorGyroI_YAW = 0;
      errorAngleI[ROLL] = 0;
      errorAngleI[PITCH] = 0;
    }
    
    imu.get_gyr_data();
    imu.get_acc_data();

    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res);

    int16_t angle[2];
    angle[ROLL] = orientation.get_roll()*180.0*10.0/PI;
    angle[PITCH] = orientation.get_pitch()*180.0*10.0/PI;

    roll_rate = roll_rate*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
    pitch_rate = pitch_rate*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
    yaw_rate = yaw_rate*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;
    
    int16_t gyroData[3];
    gyroData[ROLL] = (int16_t)(roll_rate*16.4)>>2;
    gyroData[PITCH] = (int16_t)(pitch_rate*16.4)>>2;
    gyroData[YAW] = (int16_t)(yaw_rate*16.4)>>2;

    // PITCH & ROLL
    for(axis=0;axis<2;axis++) {
      rc = rcCommand[axis]*0.1;//rcCommand[axis]*0.3//rcCommand[axis]<<1;
      error = rc - gyroData[axis];
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);       // WindUp   16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
  
      ITerm = (errorGyroI[axis]>>7)*ki>>6;                        // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
  
      PTerm = (rc*kp)>>6;

      angleCorrection[ROLL] = 30;
      angleCorrection[PITCH] = -35;

      // ANGLE MOde PID
      if(false){
        // 50 degrees max inclination
        errorAngle         = constrain(rcCommand[axis],-75,+75) - angle[axis] + angleCorrection[axis]; //16 bits is ok here
        errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);                                                // WindUp     //16 bits is ok here
  
        PTermACC           = errorAngle*level_kp>>7; // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result
  
        int16_t limit      = level_kd*5;
        PTermACC           = constrain(PTermACC,-limit,+limit);
  
        ITermACC           = errorAngleI[axis]*level_ki>>12;   // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
  
        ITerm              = ITermACC;
        PTerm              = PTermACC;
      }
  
      PTerm -= (gyroData[axis]*kp)>>6; // 32 bits is needed for calculation   
  
      delta          = gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
      lastGyro[axis] = gyroData[axis];
      DTerm          = delta1[axis]+delta2[axis]+delta;
      delta2[axis]   = delta1[axis];
      delta1[axis]   = delta;
   
      DTerm = (DTerm*kd)>>5;        // 32 bits is needed for calculation
  
      axisPID[axis] = PTerm + ITerm - DTerm;
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

    //Serial.print(gyroData[ROLL]);
    //Serial.print(" ");
    //Serial.println(gyroData[PITCH]);

    
    
    
    update_pid = 0;
    //float elapsed_time = micros() - begining_time;
    //Serial.println(elapsed_time, 0);
  }
}
