#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "PsxLib.h"

#define MULTIWII 0

RF24 radio(4, 5, 18, 19, 23); // CE, CSN, SCK, MISO, MOSI
Psx ps2x;

const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver!!!

byte prev_arm = 0;
byte prev_AUX2 = 0;

struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte arm; // AUX1
  byte AUX2;
};

MyData data;

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
  Serial.begin(115200);
  Serial.println("");

#if MULTIWII
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
#else
  radio.begin();
  //radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  //radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
#endif

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  data.arm = 0;
  data.AUX2 = 0;

  ps2x.setupPins(26, 27, 25, 32, 50);  // dataPin, cmndPin, attPin, clockPin, delay

  if(radio.isChipConnected())
  {
    Serial.println("Radio is connected!");
    Serial.println("Starting transmission!");
    digitalWrite(5, HIGH);
  }
  else
  {
    Serial.println("Radio not found!");
    digitalWrite(5, LOW);
    while(1);
  }
  
}

void loop() {

  ps2x.read();

  data.throttle = joystick_map(ps2x.analog(3), 0, 123, 255, 255, 0); // vertical left
  data.yaw = joystick_map(ps2x.analog(2), 0, 123, 255, 0, 255); // horizontal left
  data.pitch = joystick_map(ps2x.analog(1), 0, 123, 255, 255, 0); // vertical right
  data.roll = joystick_map(ps2x.analog(0), 0, 123, 255, 0, 255); // horizontal right

  uint8_t temp_arm = ps2x.button(PSB_R1);
  if( temp_arm == 1 && prev_arm == 0)
  {
    //Serial.println("Switching");
    if(data.arm == 0 && data.throttle == 0)
    {
      data.arm = 1;
    }
    else
    {
      data.arm = 0;
    }
  }
  prev_arm = temp_arm;

  uint8_t temp_aux2 = ps2x.button(PSB_L1);
  if( temp_aux2 == 1 && prev_AUX2 == 0)
  {
    data.AUX2 = !data.AUX2;
  }
  prev_AUX2 = temp_aux2;
  
  /*Serial.print(data.throttle);
  Serial.print(" ");
  Serial.print(data.yaw);
  Serial.print(" ");
  Serial.print(data.pitch);
  Serial.print(" ");
  Serial.print(data.roll);
  Serial.print(" ");
  Serial.print(data.arm);
  Serial.print(" ");
  Serial.println(data.AUX2);*/
  
  radio.write(&data, sizeof(MyData));
  //delay(10); 
}
