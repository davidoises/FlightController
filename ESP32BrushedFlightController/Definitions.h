#ifndef DEFINITIONS_h
#define DEFINITIONS_h

#include "Arduino.h"

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

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
};

//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

// RF output message structure
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

#endif
