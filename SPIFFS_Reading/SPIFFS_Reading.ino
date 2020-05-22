#include "SPIFFS.h"
 
void setup() {
 
   Serial.begin(500000);
   delay(500);
   Serial.println("\n");
 
   if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
   }

 
    //---------- Read file
    File fileToRead = SPIFFS.open("/drone.csv");
 
    if(!fileToRead){
        Serial.println("Failed to open file for reading");
        return;
    }
 
    Serial.println("File Content:");
 
    while(fileToRead.available()){
 
        Serial.write(fileToRead.read());
    }
 
    fileToRead.close();

    //SPIFFS.remove("/drone.csv");
}
 
void loop() {}
