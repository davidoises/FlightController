#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "PsxLib.h"

RF24 radio(4, 5, 18, 19, 23); // CE, CSN, SCK, MISO, MOSI
Psx ps2x;

const byte address[6] = "00001";
String switch_status = "KK";

void setup() {
  Serial.begin(115200);
  Serial.println("");
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  ps2x.setupPins(26, 27, 25, 32, 50);  // dataPin, cmndPin, attPin, clockPin, delay

  Serial.println("Starting transmission");
}

void loop() {

  ps2x.read();

  if(ps2x.button(PSB_L1) || ps2x.button(PSB_R1)) { //print stick values if either is TRUE
    
    byte msg[4];
    msg[0] = ps2x.analog(0);
    msg[1] = ps2x.analog(1);
    msg[2] = ps2x.analog(2);
    msg[3] = ps2x.analog(3);

    Serial.print(msg[0], DEC);
    Serial.print(" ");
    Serial.print(msg[1], DEC);
    Serial.print(" ");
    Serial.print(msg[2], DEC);
    Serial.print(" ");
    Serial.println(msg[3], DEC);
    
    radio.write(&msg, sizeof(msg));
  }
  /*byte msg[4] = {123, 123, 123, 132};
  radio.write(&msg, sizeof(msg));*/
  delay(40); 
}
