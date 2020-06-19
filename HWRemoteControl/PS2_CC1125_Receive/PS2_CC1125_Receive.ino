#include "CC1125.h"

CC1125 rf_comm;

// Message buffers
uint8_t rxBuffer[128] = {0};
uint8_t pkt_size = 0;

// reception ISR
bool msg_flag = false;
void msg_flag_isr()
{
  msg_flag = true; 
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  //Serial.begin(500000);
  Serial.begin(1000000);
  delay(1000);
  Serial.println("");
  Serial.println("Starting");
  
  rf_comm.begin();
  Serial.println("CC1125 Initialized!");
  rf_comm.manualCalibration();
  Serial.println("Finished calibration");
  rf_comm.receive();
  Serial.println("Available for message recpetion");

  //Antes 4 = RF INT or 15
  pinMode(15, INPUT_PULLUP);
  attachInterrupt(15, msg_flag_isr, FALLING);
}

void loop() {
  
  if(msg_flag)
  {
    rf_comm.get_packet(rxBuffer, pkt_size);
    msg_flag = false;
  }

  if(pkt_size != 0)
  {
    for(int i = 0; i < pkt_size; i++)
    {
      Serial.print(rxBuffer[i], DEC);
      Serial.print(", ");
      //Serial.print((char)rxBuffer[i]);
    }
    Serial.println("");
    pkt_size = 0;
  }
}
