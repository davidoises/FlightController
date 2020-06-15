#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define INTERRUPT_PIN 15

RF24 radio(4, 5, 18, 19, 23); // CE, CSN, SCK, MISO, MOSI
const byte address[6] = "00001";
unsigned long prev_time = 0;

bool msg_flag = false;
void msg_flag_isr()
{
  msg_flag = true; 
}


void setup() {
  Serial.begin(115200);
  
  radio.begin();
  radio.maskIRQ(1,1,0);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(INTERRUPT_PIN, msg_flag_isr, FALLING);
  
  prev_time = millis();
}

void loop() {
  //if (radio.available()) {
  if(msg_flag)
  {
    unsigned long current_time = millis();
    int dt = current_time - prev_time;
    prev_time = current_time;
    
    byte msg[4] = {0};
    radio.read(&msg, sizeof(msg));

    Serial.print(msg[0], DEC);
    Serial.print(" ");
    Serial.print(msg[1], DEC);
    Serial.print(" ");
    Serial.print(msg[2], DEC);
    Serial.print(" ");
    Serial.println(msg[3], DEC);
    
    /*Serial.print(dt);
    Serial.print(" ");
    Serial.println(String(text));*/
    delay(40);
    msg_flag = false;
  }
}
