#include "SPIFFS.h"

#define RXD2 16
#define TXD2 15

// Data logging
File logger;

String ten_lines[100];
int serial_counter = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial.println("");
  Serial.println("Beginning Data Logging");
  delay(100);
  //Serial2.begin(500000, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);

  // Start SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  logger = SPIFFS.open("/drone.csv", FILE_WRITE);
  if (!logger) {
    Serial.println("There was an error opening the file for writing");
    return;
  }
  logger.println("Sync, Step, roll, roll_rate_setpoint, roll_rate");
}

void loop() {
  String ins = "";

  for(int i = 0; i<10; i++)
  {
    ins += Serial2.readStringUntil('\n');
    ins += "\n";
    
    //serial_counter = (serial_counter+1)%9;
  }
  logger.print(ins);
  Serial.print(ins);
}
