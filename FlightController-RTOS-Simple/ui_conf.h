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

BLYNK_WRITE(V0)
{
  eStop = param.asInt(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V1)
{
  throttle = constrain(param.asDouble(), 1000.0, 2000.0); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V2)
{
  roll = constrain(param.asDouble(), 1000.0, 2000.0); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V3)
{
  pitch = constrain(param.asDouble(), 1000.0, 2000.0); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V4)
{
  yaw = constrain(param.asDouble(), 1000.0, 2000.0); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V5)
{
  
  //alt_hold = param.asInt(); // assigning incoming value from pin V1 to a variable
  //alt_callback = 1;
  if(param.asInt() == 1)
  {
    aux1 = 1500;  
  }
  else{
    aux1 = 1000;  
  }
  ui_callback = 1;
  
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
