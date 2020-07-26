#define MAIN_EXECUTION

#include "BMX055.h"
#include "MS5611.h"
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "Estimates.h"
#include "PID.h"
#include "Definitions.h"

// Class objects for data acquisition and sensor fusion
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
MS5611 altimeter = MS5611();

// Controller data i.e. setpoints for roll, pitch, yaw, throttle and mode selections
received_message controller_data;

// Telemetry sctruct and transmission flag
sent_message drone_data;
uint8_t update_telemetry;

// RF interface variables
unsigned long prev_rf_time = 0;
uint8_t prev_stop = 0;
uint8_t prev_mode = 0;
uint8_t eStop = 1;
uint8_t altHold = 0;
uint8_t altHoldStart = 0;
float rcCommand[4];

// Sensor calibration flags and acc offsets
uint8_t calibrate_alt;
uint8_t calibrate_acc = 0;
float acc_calibration[3] = {0,0,0};

// Orientation measurements
float gyroLPF[3] = {0,0,0};
float roll = 0;
float pitch = 0;

// Vertical acceleration
float acczSmooth = 0;
int32_t BaroPID = 0;
int32_t errorVelocityI = 0;
int32_t EstAlt = 0;
int32_t AltitudeSetpoint = 0;
int16_t initalThrottle = 0;

// Attitude PID calculation variables
unsigned long prev_pid_time = 0;
float attitudePID[3] = {0,0,0};

// Motor PWM values
int16_t motor[4];

// Thread handles for tasks information display
TaskHandle_t rf_handle = NULL;

// RF reception callback and RF loop flag
uint8_t msg_flag;
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&controller_data, incomingData, sizeof(controller_data));
  msg_flag = 1;
}

// PID timer, PID ISR and PID loop flag
hw_timer_t * pid_timer = NULL;
volatile uint8_t update_pid;
void IRAM_ATTR pid_isr() {
  update_pid = 1;
}

/*
 * RF processing loop
 * This loop handles incoming data, adjusts it to standard scales i.e. (-500, 500)
 * RF failsafe implementation is programmed here
 */
void rfLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {

    // Failsafe mechanism, if RF loop time is greater than 500 ms turn off everything
    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_rf_time;
    if(dt > 500)
    {
      eStop = 1;
    }
    
    // Message processing onnly on message reception
    if(msg_flag)
    {
      // Update RF loop time
      prev_rf_time = current_time;

      // Joystick reading and scaling
      rcCommand[THROTTLE] = map(controller_data.vl_stick, 0, 254, 1000, 2000); // vertical left
      rcCommand[ROLL] = map(controller_data.hr_stick, 0, 254, -500, 500); // horizontal right
      rcCommand[PITCH] = map(controller_data.vr_stick, 0, 254, -500, 500); // vertical right
      rcCommand[YAW] = map(controller_data.hl_stick, 0, 254,-500, 500); // horizontal left
      
      // Example of exponential scaling to RC data
      //x = ((float)abs(rcData[PITCH]))/128.0;
      //rcCommand[PITCH] = ((1526.0 + 65.0*(x*x-15.0))*90.0*x)/1192.0;
      //if(rcData[PITCH] < 0) rcCommand[PITCH] = -rcCommand[PITCH];

      // Emergency stop button simulated as a switch
      uint8_t temp_stop = controller_data.r_back_button;
      if(temp_stop == 1 && prev_stop == 0)
      { 
        // Only disable emergency stop if throttle is at the lowest value
        if(eStop == 1 && rcCommand[THROTTLE] == 1000)
        {
          eStop = 0;
        }
        else
        {
          eStop = 1;
        }
      }
      prev_stop = temp_stop;

      // AltHold button simulated as a switch
      uint8_t temp_mode = controller_data.l_back_button;
      if(temp_mode == 1 && prev_mode == 0)
      {
        altHold = !altHold;
        // set alHoldStart flag just once and exactly when althold was enabled
        if(altHold)
        {
          altHoldStart = 1;
        }
      }
      prev_mode = temp_mode;

      // Use blue led to indicate if high(ON) or low(OFF) throttle values are received
      if(rcCommand[THROTTLE] > 1550)
      {
        digitalWrite(LED_PIN, HIGH);
      }
      else
      {
        digitalWrite(LED_PIN, LOW);
      }
      
      msg_flag = 0;
    }

    // If the PID loop set the flag, transmit the data calculated on the PID looop
    if(update_telemetry)
    {
      esp_now_send(broadcastAddress, (uint8_t *) &drone_data, sizeof(drone_data));
      update_telemetry = 0;
    }
    
    // Minimum delay of 4ms between polling
    vTaskDelay(4);
  }
}

/*
 * setup function
 * This function initialzes all the subsystems required for proper functionality
 * Communication, Sensors, PWM output, RF loop and PID loop settings are initialzed here
 */
void setup(void)
{
  // Start UART Bus @115200 bps
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting system");
  
  // Start SPI Bus @400 kbps
  Wire.begin();
  Wire.setClock(400000);

  // Initialize virtual EEPROM on a fixed size (size of acc calibration values)
  EEPROM.begin(EEPROM_SIZE);

  // Initialize PWM channels
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA);
  ledcWrite(ledChannelA, 0);

  ledcSetup(ledChannelB, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, ledChannelB);
  ledcWrite(ledChannelB, 0);

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC);
  ledcWrite(ledChannelC, 0);

  ledcSetup(ledChannelD, FREQ, RESOLUTION);
  ledcAttachPin(PWMD, ledChannelD);
  ledcWrite(ledChannelD, 0);

  // During setup the blue led is held high
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // small delay to avoid noise measurment after turning on
  //TODO: Could possibly use a controller button to allow start of setup
  delay(2000);

  // IMU intialization (Gyro automatic calibration happens here)
  imu.acc_init();
  imu.gyr_init();

  // Acc calibration happens continoulsy on the PID loop for N(512) samples, this is just a flag to enable calibration
  //calibrate_acc = 1;

  // Read previously stored calibration values. If calibration flag is set this values will be ignores and set to new calibration values after calculation
  int address = 0;
  EEPROM.get(address, acc_calibration);

  // Altimeter initialization and calibration
  altimeter.begin(MS5611_ULTRA_HIGH_RES);
  altimeter.requestTemperature(); // Request pressure so it is already available for first measurement

  // Baro calibration happens continoulsy on the PID loop for N samples, this is just a flag to enable calibration
  calibrate_alt = 1;
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Callback for message recepion
  esp_now_register_recv_cb(OnDataRecv);

  // Generate an ESP now receiver peer reference
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Register peer on ESP Now objects, this doesnt require the peer to be on      
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // Free bytes for RTOS stacks management
  //Serial.println(ESP.getFreeHeap());
  //total heap = 177360 bytes?

  // Orientation PID timer, set to PID_SAMPLING microseconds
  pid_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pid_timer, &pid_isr, true);
  timerAlarmWrite(pid_timer, PID_SAMPLING, true);
  
  // RF loop RTOS task running on core 0. This avoids any disruptions on PID loop
  xTaskCreatePinnedToCore(
    rfLoop,      /* Function to implement the task */
    "RF core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &rf_handle,           /* Task handle. */
    0);             /* Core where the task should run */

  // Calibration has finished, turn led OFF
  digitalWrite(LED_PIN, LOW);
  
  // Start pid timer
  timerAlarmEnable(pid_timer);
  
  Serial.println("Intial setup finished");

  // Initialaze time regerences for loop period calculations
  prev_pid_time = micros();
  prev_rf_time = millis();
}

/*
 * PID loop
 * This loop reads sensor values, calculates PID control law and applies to motors
 * Additionaly stores calculated data on RF structure for transmission
 */
void loop(void)
{
  static uint8_t taskOrder=0;
  
  // only run when the timer sets the flag to start
  if(update_pid)
  {
    // Processing time calculation - start of timer
    unsigned long begining_time = micros();
    
    // Loop time calculation: should be exactly PID_SAMPLING microseconds
    unsigned long current_time = micros();
    float dt = current_time - prev_pid_time;
    prev_pid_time = current_time;
    
    // Perform extra measurements on lower frecuencies than the main loop timer
    switch(taskOrder)
    {
      // Read baro data (Pressure and temperature alternating readings)
      case 0:
        taskOrder++;
        if(get_baro_data() != 0)break;
        
      // Calculate altitude and althold PID
      case 1:
        taskOrder=0;
        altitude_estimation();
        break;
    }

    // If Altitude hold was just enabled reset its PID and get sepoint
    if(altHoldStart)
    {
      altHoldStart = 0;
      errorVelocityI = 0;
      BaroPID = 0;
      initalThrottle  = rcCommand[THROTTLE];
      
      AltitudeSetpoint = EstAlt;
    }
    
    // Get IMU data
    get_gyr_compensated_data();
    get_acc_compensated_data();

    // Gyro low pass filter for gyro PID
    get_gyr_lpf();

    // Estimate euler angles, just used to get world frame acceleration
    attitude_estimation(dt);
    
    // Get world frame z acceleration for AltHold PID
    acceleration_estimation(dt);

    // Scale/sensitivity adjustment for attitude setpoints
    float setpoints[3];
    setpoints[ROLL] = rcCommand[ROLL]*0.3/4.1;
    setpoints[PITCH] = rcCommand[PITCH]*0.3/4.1;
    setpoints[YAW] = rcCommand[YAW]*1.5/4.1;

    // Calculate roll, pitch and yaw pids
    calculateAttitudePID(setpoints);

    // If altitude hold is on, include its PID on the mixer
    if(altHold)
    {
      //rcCommand[THROTTLE] = 1590;// Discharged battery
      //rcCommand[THROTTLE] = 1540;// Full battery battery
      rcCommand[THROTTLE] = 1565 + BaroPID;
      //rcCommand[THROTTLE] = initalThrottle + BaroPID;
    }

    // Mix table
    motor[0] = rcCommand[THROTTLE] - attitudePID[ROLL] + attitudePID[PITCH] + attitudePID[YAW];
    motor[1] = rcCommand[THROTTLE] + attitudePID[ROLL] + attitudePID[PITCH] - attitudePID[YAW];
    motor[2] = rcCommand[THROTTLE] + attitudePID[ROLL] - attitudePID[PITCH] + attitudePID[YAW];
    motor[3] = rcCommand[THROTTLE] - attitudePID[ROLL] - attitudePID[PITCH] - attitudePID[YAW];

    // Get the max PWM to be generated for a single mmotor
    int16_t maxMotor=motor[0];
    for(uint8_t i=1; i< 4; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
      
    for(uint8_t i=0; i< 4; i++) {
      
      // if any motor saturates (exceeds 2000 PWM), decrease all 4 motors by the same amount to maintain their relations and avoid saturation
      if (maxMotor > 2000)
        motor[i] -= maxMotor - 2000;
      motor[i] = constrain(motor[i], 1000, 2000);
      
      // If not armed just turn off everything
      if(rcCommand[THROTTLE] < 1100 || eStop)
      {
        motor[i] = 1000;
        resetAttitudePID();
      }
    }

    // Map ESC PWMs to brushed motors PWMs
    for(uint8_t i=0; i< 4; i++) {
      motor[i] = map(motor[i], 1000, 2000, 0, MAX_PWM);
    }

    // Output the actual PWMs to the motors
    ledcWrite(ledChannelA, motor[0]);
    ledcWrite(ledChannelB, motor[1]);
    ledcWrite(ledChannelC, motor[2]);
    ledcWrite(ledChannelD, motor[3]);

    // Log RTOS stack usage
    //Serial.print(uxTaskGetStackHighWaterMark(blynk_handle));
    //Serial.print("\t");
    //Serial.print(uxTaskGetStackHighWaterMark(imu_handle));
    
    // Clear the flag, to allow next execution signaling
    update_pid = 0;

    // Processing time calculation - end of timer
    float elapsed_time = micros() - begining_time;

    // Telemetry update
    // Send all this through the RF packet
    drone_data.loop_time = dt;
    drone_data.process_time = elapsed_time;

    drone_data.roll_rate_setpoint = rcCommand[ROLL]*0.3/4.1;
    drone_data.pitch_rate_setpoint = rcCommand[PITCH]*0.3/4.1;
    drone_data.yaw_rate_setpoint = rcCommand[YAW]*1.5/4.1;
    drone_data.altitude_setpoint = AltitudeSetpoint;
    drone_data.roll_rate = gyroLPF[ROLL];
    drone_data.pitch_rate = gyroLPF[PITCH];
    drone_data.yaw_rate = gyroLPF[YAW];
    drone_data.altitude = EstAlt;
    
    // This flag tells RF Loop to send data. Set this to 0 if not needed to send data
    update_telemetry = 1;
  }
}
