#include "CC1125.h"

CC1125 rf_comm;

// Message buffers
uint8_t rxBuffer[128] = {0};
uint8_t txBuffer[128] = {0};
uint8_t pkt_size = 0;

// Message sync
bool msg_flag = false;
int counter = 0;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  Serial.begin(500000);
  delay(500);
  Serial.println("\nStarting");
  
  rf_comm.begin();
  Serial.println("CC1125 Initialized!");
  rf_comm.manualCalibration();
  Serial.println("Finished calibration");
}

void loop() {

  while(Serial.available() > 0)
  {
    char mychar = Serial.read();
    if(mychar == ':')
    {
      msg_flag = true;
      txBuffer[1] = 0xAA;// Seems not important
      counter = 0;
    }

    else if(msg_flag && mychar == ';')
    {
      txBuffer[0] = counter+1;
      for(int i = 1; i < counter+1; i++)
      {
        Serial.print((char)txBuffer[i+1]);
      }
      Serial.println("");
      rf_comm.sendPacket(txBuffer, counter+2);
      msg_flag = false;
    }
    else if(msg_flag)
    {
      counter++;
      txBuffer[counter+1] = mychar;
    }
  }
}
