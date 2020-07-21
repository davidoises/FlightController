#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define INTERRUPT_PIN 15

struct RF24Data {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
  byte switches;
};

const uint64_t pipeIn = 0xE8E8F0F0E1LL;  //Remember, SAME AS TRANSMITTER CODE

RF24 radio(4, 5, 18, 19, 23); // CE, CSN, SCK, MISO, MOSI

RF24Data MyData;
unsigned long prev_time = 0;

bool msg_flag = false;
void msg_flag_isr()
{
  msg_flag = true; 
}


void setup() {
  //Serial.begin(115200);
  Serial.begin(1000000);
  
  radio.begin();
  radio.maskIRQ(1,1,0);
  //radio.setDataRate(RF24_250KBPS);
  //radio.setAutoAck(false);                    // Ensure autoACK is enabled
  
  radio.openReadingPipe(0, pipeIn);
  //radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  

  //Serial.println(radio.isChipConnected());
  
  radio.printDetails();
  Serial.println(radio.isChipConnected());

  delay(1000);

  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(INTERRUPT_PIN, msg_flag_isr, FALLING);
  
  prev_time = millis();
}

void loop() {
  if (radio.available())
  //if(msg_flag)
  {
    unsigned long current_time = millis();
    int dt = current_time - prev_time;
    prev_time = current_time;
    
    radio.read(&MyData, sizeof(RF24Data));
    //Serial.print(dt);
    //Serial.print(" ");
    Serial.print(MyData.throttle);
    Serial.print(" ");
    Serial.print(MyData.yaw);
    Serial.print(" ");
    Serial.print(MyData.pitch);
    Serial.print(" ");
    Serial.print(MyData.roll);
    Serial.print(" ");
    Serial.print(MyData.AUX1);
    Serial.print(" ");
    Serial.println(MyData.AUX2);
    
    msg_flag = false;
  }
  
}
