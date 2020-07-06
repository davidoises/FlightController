#include <esp_now.h>
#include <WiFi.h>

uint8_t droneAddress[] = {0xD8, 0xA0, 0x1D, 0x5D, 0xFB, 0x04};
uint8_t drone2Address[] = {0xD8, 0xA0, 0x1D, 0x55, 0xB3, 0xE8};

// RF input message structure
typedef struct sent_message {
  float loop_time;
  float process_time;
  /*
  float acc_x;
  float acc_y;
  float acc_z;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  */
  float pitch;
  float roll;
} sent_message;

sent_message drone_data;


unsigned long prev_time = 0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  bool drone1 = true;
  bool drone2 = true;
  for(uint8_t i = 0; i < 6; i++)
  {
    if(mac[i] != droneAddress[i])
    {
      drone1 = false;
      break;
      //return;
    }
    //Serial.println(mac[i], HEX);
  }

  if(!drone1)
  {
    for(uint8_t i = 0; i < 6; i++)
    {
      if(mac[i] != drone2Address[i])
      {
        drone2 = false;
        break;
        //return;
      }
    }
  }

  if(!drone1 && !drone2)
  {
    return;
  }
  
  memcpy(&drone_data, incomingData, sizeof(drone_data));


  //Serial.print(drone_data.loop_time);
  //Serial.print(" ");
  //Serial.println(drone_data.process_time);
  //Serial.print(" ");
  
  //Serial.print(drone_data.acc_x);
  //Serial.print(" ");
  //Serial.print(drone_data.acc_y);
  //Serial.print(" ");
  //Serial.println(drone_data.acc_z);

  Serial.print(drone_data.roll*180.0/PI);
  Serial.print(" ");
  Serial.println(drone_data.pitch*180.0/PI);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  prev_time = millis();
}
 
void loop() {
}
