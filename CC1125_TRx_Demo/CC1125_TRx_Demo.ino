#include "CC1125.h"
#define RX 15
#define TX 9

CC1125 rf_comm;

// Message buffers
uint8_t rxBuffer[128] = {0};
uint8_t txBuffer[128] = {0};
uint8_t pkt_size = 0;

// reception ISR
bool msg_flag = false;
int counter = 0;

int mydata = 0;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  Serial.begin(500000);
  delay(500);
  Serial2.begin(500000, SERIAL_8N1, RX, TX);
  Serial2.println("TEST");
  
  rf_comm.begin();
  Serial.println("CC1125 Initialized!");
  rf_comm.manualCalibration();
  Serial.println("Finished calibration");
  rf_comm.receive();
  Serial.println("Available for message recpetion");

  //Antes 4 = RF INT or 15
  pinMode(4, INPUT_PULLUP);
  //attachInterrupt(4, msg_flag_isr, FALLING);

  //timer = timerBegin(0, 80, true);
  //timerAttachInterrupt(timer, &onTimer, true);
  //timerAlarmWrite(timer, 500000, true);
  //timerAlarmEnable(timer);
}

void loop() {

  //if(reached_timer)
  while(Serial2.available() > 0)
  {
    char mychar = Serial2.read();
    if(mychar == ':')
    {
      msg_flag = true;
      txBuffer[1] = 0xAA;// Seems not important
      counter = 0;
    }

    else if(msg_flag && mychar == ';')
    {
      txBuffer[0] = counter;
      //Serial.println("");
      //Serial.println("Sent");
      for(int i = 0; i < counter+1; i++)
      {
        Serial.println(txBuffer[counter+1]);
      }
      rf_comm.sendPacket(txBuffer, counter+1);
      msg_flag = false;
    }
    else if(msg_flag)
    {
      counter++;
      txBuffer[counter+1] = mychar;
      //Serial.print(mychar);
    }
  }
  
  /*if(msg_flag)
  {
    rf_comm.get_packet(rxBuffer, pkt_size);
    msg_flag = false;
  }

  if(pkt_size != 0)
  {
    for(int i = 0; i < pkt_size; i++)
    {
      Serial.print("0x");
      Serial.print(rxBuffer[i], HEX);
      Serial.print(", ");
    }
    Serial.println("");
    pkt_size = 0;
  }
  */  
}
