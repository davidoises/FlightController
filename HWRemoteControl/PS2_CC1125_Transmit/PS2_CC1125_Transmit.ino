#include "CC1125.h"
#include "PsxLib.h"

CC1125 rf_comm;
Psx ps2x;

// Message buffers
uint8_t txBuffer[128] = {0};
uint8_t pkt_size = 0;

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

  if(ps2x.button(PSB_L1) || ps2x.button(PSB_R1)) { //print stick values if either is TRUE
    txBuffer[1] = 0xAA; // Receiver address  
    txBuffer[2] = ps2x.analog(0);
    txBuffer[3] = ps2x.analog(1);
    txBuffer[4] = ps2x.analog(2);
    txBuffer[5] = ps2x.analog(3);

    uint8_t len = 6;
    txBuffer[0] = len -1;

    rf_comm.sendPacket(txBuffer, len);

    /*Serial.print(txBuffer[2]);
    Serial.print(" ");
    Serial.print(txBuffer[3]);
    Serial.print(" ");
    Serial.print(txBuffer[4]);
    Serial.print(" ");
    Serial.println(txBuffer[5]);*/
  }
  delay(40);
}
