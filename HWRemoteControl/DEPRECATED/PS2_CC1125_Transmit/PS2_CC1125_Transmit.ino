#include "CC1125.h"
#include "PsxLib.h"

CC1125 rf_comm;
Psx ps2x;

// Message buffers
uint8_t txBuffer[128] = {0};
uint8_t pkt_size = 0;

int joystick_map(int value, float low_input, float middle_input, float high_input, float low_output, float high_output)
{
  float middle_output = (low_output + high_output)/2.0;
  if(value <= middle_input)
  {
    return map(value, low_input, middle_input, low_output, middle_output);
  }
  else
  {
    return map(value, middle_input, high_input, middle_output, high_output);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.begin(500000);
  delay(1000);
  Serial.println("");
  Serial.println("Starting");

  rf_comm.begin();
  Serial.println("CC1125 Initialized!");
  rf_comm.manualCalibration();
  Serial.println("Finished calibration");
  
  ps2x.setupPins(26, 27, 25, 32, 50);  // dataPin, cmndPin, attPin, clockPin, delay
  Serial.println("PS2 Controller Initialized");
}

void loop() {

  ps2x.read();

  uint8_t len = 16;
  txBuffer[0] = len -1;
  txBuffer[1] = 0xAA; // Receiver address  

  // Joystick values
  txBuffer[2] = joystick_map(ps2x.analog(0), 0, 123, 255, 0, 255); // horizontal right
  txBuffer[3] = joystick_map(ps2x.analog(1), 0, 123, 255, 255, 0); // vertical right
  txBuffer[4] = joystick_map(ps2x.analog(2), 0, 123, 255, 0, 255); // horizontal left
  txBuffer[5] = joystick_map(ps2x.analog(3), 0, 123, 255, 255, 0); // vertical left

  txBuffer[6] = ps2x.button(PSB_START);
  txBuffer[7] = ps2x.button(PSB_SELECT);
  
  txBuffer[8] = ps2x.button(PSB_TRIANGLE);
  txBuffer[9] = ps2x.button(PSB_CIRCLE);
  txBuffer[10] = ps2x.button(PSB_CROSS);
  txBuffer[11] = ps2x.button(PSB_SQUARE);

  txBuffer[12] = ps2x.button(PSB_L3); // Left stick press
  txBuffer[13] = ps2x.button(PSB_R3); // Right stick press

  txBuffer[14] = ps2x.button(PSB_L1); // Left back button
  txBuffer[15] = ps2x.button(PSB_R1); // Right back button

  rf_comm.sendPacket(txBuffer, len);
  delay(40);
}
