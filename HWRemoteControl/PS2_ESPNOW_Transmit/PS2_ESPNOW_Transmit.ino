#include "PsxLib.h"
#include <esp_now.h>
#include <WiFi.h>

Psx ps2x;

uint8_t receiverAddress[] = {0x84, 0x0D, 0x8E, 0x19, 0xE4, 0xA8};

typedef struct struct_message {
  uint8_t hr_stick;
  uint8_t vr_stick;
  uint8_t hl_stick;
  uint8_t vl_stick;
  uint8_t start;
  uint8_t select;
  uint8_t triangle;
  uint8_t circle;
  uint8_t cross;
  uint8_t square;
  uint8_t l_stick_button;
  uint8_t r_stick_button;
  uint8_t l_back_button;
  uint8_t r_back_button;
} struct_message;

struct_message controller_data;

int joystick_map(int value, float low_input, float middle_input, float high_input, float low_output, float high_output)
{
  float middle_output = (low_output + high_output)/2.0;
  if(value <= middle_input)
  {
    return map(value, low_input, middle_input, low_output, middle_output);
  }
  else
  {
    return map(value, middle_input, high_input, middle_output, high_output);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESPNow initialized correctly");
  
  ps2x.setupPins(26, 27, 25, 32, 50);  // dataPin, cmndPin, attPin, clockPin, delay
  Serial.println("PS2 Controller Initialized");
}

void loop() {

  ps2x.read();

  // Joystick values
  controller_data.hr_stick = joystick_map(ps2x.analog(0), 0, 123, 255, 0, 255); // horizontal right
  controller_data.vr_stick = joystick_map(ps2x.analog(1), 0, 123, 255, 255, 0); // vertical right
  controller_data.hl_stick = joystick_map(ps2x.analog(2), 0, 123, 255, 0, 255); // horizontal left
  controller_data.vl_stick = joystick_map(ps2x.analog(3), 0, 123, 255, 255, 0); // vertical left

  controller_data.start = ps2x.button(PSB_START);
  controller_data.select = ps2x.button(PSB_SELECT);
  
  controller_data.triangle = ps2x.button(PSB_TRIANGLE);
  controller_data.circle = ps2x.button(PSB_CIRCLE);
  controller_data.cross = ps2x.button(PSB_CROSS);
  controller_data.square = ps2x.button(PSB_SQUARE);

  controller_data.l_stick_button = ps2x.button(PSB_L3); // Left stick press
  controller_data.r_stick_button = ps2x.button(PSB_R3); // Right stick press

  controller_data.l_back_button = ps2x.button(PSB_L1); // Left back button
  controller_data.r_back_button = ps2x.button(PSB_R1); // Right back button

  esp_now_send(receiverAddress, (uint8_t *) &controller_data, sizeof(controller_data));

  delay(40);
}
