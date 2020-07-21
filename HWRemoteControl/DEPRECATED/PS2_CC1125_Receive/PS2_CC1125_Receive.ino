#include "CC1125.h"

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 12
#define VBAT_ADC 27
#define IMU_INTERRUPT 25
#define RF_INTERRUPT 15
#define LED_PIN 0

// setting PWM properties
#define FREQ 60000
#define RESOLUTION 10
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

#define MAX_PWM (1<<RESOLUTION)-1

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

float throttle = 0;

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  //Serial.begin(500000);
  Serial.begin(1000000);
  delay(1000);
  Serial.println("");
  Serial.println("Starting");

  // Initialize PWM channels
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA); // negative torque
  ledcWrite(ledChannelA, 0);
  
  rf_comm.begin();
  Serial.println("CC1125 Initialized!");
  rf_comm.manualCalibration();
  Serial.println("Finished calibration");
  rf_comm.receive();
  Serial.println("Available for message recpetion");

  //Antes 4 = RF INT or 15
  pinMode(RF_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(RF_INTERRUPT, msg_flag_isr, FALLING);
}

void loop() {
  
  if(msg_flag)
  {
    rf_comm.get_packet(rxBuffer, pkt_size);
    msg_flag = false;
  }

  if(pkt_size != 0)
  {
    /*for(int i = 0; i < pkt_size; i++)
    {
      Serial.print(rxBuffer[i], DEC);
      Serial.print(", ");
      //Serial.print((char)rxBuffer[i]);
    }
    Serial.println("");*/
    throttle = map_float(rxBuffer[3], 0, 255, 0, 850); // vertical left
    Serial.println(throttle);
    ledcWrite(ledChannelA, throttle);
    pkt_size = 0;
  }
}
