#ifndef UI_CONF
#define UI_CONF

//#define BLYNK_PRINT Serial

#define WIFI_GUI 0


#if WIFI_GUI
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#else
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#endif



// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "eYcrTSn-5ZZDlGWLoPxtSGaHWEAMGeNb";

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Cablevision127";
//char pass[] = "Contra_Word.";
//char ssid[] = "IZZI28681";
//char pass[] = "37662DCDAF5B0E8E";
//char ssid[] = "IZZI28681";
//char pass[] = "37662DCDAF5B0E8E";
//char ssid[] = "INFINITUM0C3E71";
//char pass[] = "06653252FF";

// Global variables related to user interface
uint8_t ui_callback = 0;
uint8_t alt_callback = 0;
uint8_t alt_hold = 0;
double original_throttle = 0;
BLYNK_WRITE(V0)
{
  eStop = param.asInt(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V1)
{
  throttle = constrain(param.asInt(), 1000, 1800); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V2)
{
  //kp_pitch = param.asDouble(); // assigning incoming value from pin V1 to a variable
  //kp_yaw = param.asDouble()/1000.0;
  kp_roll = param.asDouble()/100.0;
  kp_pitch = 1.08*kp_roll;
  //kp_vel_z = param.asDouble();
  //roll_setpoint = param.asDouble();
  ui_callback = 1;
}

BLYNK_WRITE(V3)
{
  //ki_pitch = param.asDouble(); // assigning incoming value from pin V1 to a variable
  //ki_yaw = param.asDouble()/1000.0;
  ki_roll = param.asDouble()/100.0;
  ki_pitch = ki_roll;
  //ki_vel_z = param.asDouble();
  //kp_vel_z = param.asDouble();
  //pitch_setpoint = param.asDouble();
  ui_callback = 1;
}

BLYNK_WRITE(V4)
{
  //kd_pitch = param.asDouble(); // assigning incoming value from pin V1 to a variable
  //kd_yaw = param.asDouble()/1000.0;
  kd_roll = param.asDouble()/100.0;
  kd_pitch = kd_roll;
  //kd_vel_z = param.asDouble();
  //yaw_setpoint = param.asDouble();
  ui_callback = 1;
  //ka = param.asDouble();
}

BLYNK_WRITE(V5)
{
  
  alt_hold = param.asInt(); // assigning incoming value from pin V1 to a variable
  alt_callback = 1;
}

BLYNK_WRITE(V6)
{
  kr = param.asDouble(); // assigning incoming value from pin V1 to a variable
  //ka = param.asDouble();
  //kp_pos_z = param.asDouble();
}

BLYNK_WRITE(V7)
{
  if(param.asInt() == 1)
  {
    original_throttle = throttle;
    throttle = 1400;
  }
  else
  {
    throttle = original_throttle;
  }
}

/*BLYNK_CONNECTED()
{
  Blynk.virtualWrite(V0, eStop);
  Blynk.virtualWrite(V1, throttle);
  Blynk.virtualWrite(V2, kp_pitch);
  Blynk.virtualWrite(V3, ki_pitch);
  Blynk.virtualWrite(V4, kd_pitch);
}*/

#endif
