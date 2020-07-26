#ifndef DEFINITIONS_h
#define DEFINITIONS_h

#include "Arduino.h"

// This allows to extern variables when included from other files and to initialize variables when included from the MAIN_EXECUTION file (*.ino file)
#ifdef MAIN_EXECUTION
#define EXTERN
#define _INIT(x) = x
#define INITIALIZE
#else
#define EXTERN extern
# define _INIT(x)
#endif

/********* Global Macro definitions *************/

// EEPROM onyl holds 3 float values whcih represent 3 axis acc offsets
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

// Conversion for PWM generation
#define MAX_PWM (1<<RESOLUTION)-1
//#define DUTY_TO_PWM(x) ((float)x)*((float)MAX_PWM)/100.0

// PID sampling time
#define PID_SAMPLING 2800

/********* Global Enums and Structs *************/

// General enum for arrays holding tri-axial data
enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
};

// RF message structure received from remote controller
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

// RF message structure sent from the drone to telemetry receiver
typedef struct sent_message {
  float loop_time;
  float process_time;
  int16_t roll_rate_setpoint;
  int16_t pitch_rate_setpoint;
  int16_t yaw_rate_setpoint;
  int32_t altitude_setpoint;
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
  float altitude;
} sent_message;

/********* Global variables *************/
// This initializes the array only once. For the rest of the times this file is included, this externs the variable
#ifdef INITIALIZE
  // This can be changed to the specific MAC address of the telemetry receiver
  // ESPNOW broadcasts (send to everyone) when address is al 0xFF
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#else
  EXTERN uint8_t broadcastAddress[6];// _INIT({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
#endif

//PID constants
EXTERN float rate_rollpitch_kp _INIT(12.0*4.1/64.0);              //5;//12;
EXTERN float rate_rollpitch_ki _INIT(30.0*4.1/(64.0*128.0));      //17;//30;
EXTERN float rate_rollpitch_kd _INIT(23.0*3.0*4.1/32.0);          //52;//23;

EXTERN float rate_yaw_kp _INIT(4.1*12.0/64.0);                    //5;//12;
EXTERN float rate_yaw_ki _INIT(4.1*45.0/8192.0);

#endif
