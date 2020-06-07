// UI Control variables
double eStop = 1;
double throttle = 1000.0;
double kp = 0;
double kd = 0;
double ki = 0;

#include "ui_conf.h"

#define COMPENSATE 0

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10
#define SERVO1_X 13
#define SERVO1_Y 12
#define SERVO2_X 26
#define SERVO2_Y 25
#define ADCA 37
#define ADCB 38
#define ADCC 34
#define ADCD 35

// setting PWM properties
#define FREQ 250
#define RESOLUTION 12
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((double)FREQ)
#define MS_TO_PWM(x) ((double)x)*((double)MAX_PWM)/(((double)MAX_PERIOD)*1000.0f)

#define ledChannel ledChannelB

// Attitude sampling
#define ORIENTATION_SAMPLING 0.004f

// timer variables
volatile uint8_t update_orientation;
hw_timer_t * orientation_timer = NULL;

// Timer ISR
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
}

// Battery voltage variable
double vbat = 0;

// Measured speed
double rpm = 0;

// Loop preescaler
double presc = 0;

void setup() {
  Serial.begin(500000);
  //Serial.begin(1000000);
  
  delay(100);
  Serial.println("\r\n");
  delay(100);

  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA); // negative torque
  ledcWrite(ledChannelA, MS_TO_PWM(1000));

  ledcSetup(ledChannelB, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, ledChannelB); // negative torque
  ledcWrite(ledChannelB, MS_TO_PWM(1000));

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC); // Positive toruqe
  ledcWrite(ledChannelC, MS_TO_PWM(1000));

  ledcSetup(ledChannelD, FREQ, RESOLUTION);
  ledcAttachPin(PWMD, ledChannelD); // Positive toruqe
  ledcWrite(ledChannelD, MS_TO_PWM(1000));

  #if WIFI_GUI
  Blynk.begin(auth, ssid, pass);
  #else
  Blynk.begin(auth);
  #endif

  vbat = analogRead(37)*(3.3/4095.0)*(1.0076) + 0.1729;
  rpm = analogRead(ADCB)*33.0/4095.0;

  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);
  timerAlarmEnable(orientation_timer);
}

void loop() {
  
  Blynk.run();

  if(eStop)
  {
    ledcWrite(ledChannelA, MS_TO_PWM(1000));
    ledcWrite(ledChannelB, MS_TO_PWM(1000));
    ledcWrite(ledChannelC, MS_TO_PWM(1000));
    ledcWrite(ledChannelD, MS_TO_PWM(1000));
  }
  
  if(update_orientation)
  {
    // Battery lpf
    double temp_meas = analogRead(37)*(3.3/4095.0)*(1.0076) + 0.1729;
    vbat = 0.92*vbat + 0.08*temp_meas;

    // rpm lpf
    rpm = 0.92*rpm + 0.08*analogRead(ADCB)*33.0/4095.0;

    if(presc == 75)
    {
      if(vbat > 1.7 && !eStop)
      {
        Serial.print(vbat);
        Serial.print(" ");
        Serial.println(rpm);
      }
      else
      {
        eStop = 1;
      }
      presc = 0;
    }
    
    // Battery compensation
#if COMPENSATE
    /*
    double required_compensation = 0;
    if((2.3 - vbat) < 0.11 && (2.3 - vbat) > 0.1)
    {
      //required_compensation = (2.3 - vbat)*7.0 - 0.7105;
      required_compensation = (2.3 - vbat)*6.0 - 0.6006;
      //required_compensation = (2.3 - vbat)*5.2 - 0.5125;
    }
    else if((2.3 - vbat) > 0.11 && (2.3 - vbat) < 0.5)
    {
      //required_compensation = (2.3 - vbat)*0.6104 - 0.0075;
      //required_compensation = (2.3 - vbat)*0.59 - 0.0054;
      required_compensation = (2.3 - vbat)*0.599 - 0.0062;
    }
    else if((2.3 - vbat) > 0.5)
    {
      //required_compensation = (2.3 - vbat)*1.5 - 0.455;
      //required_compensation = (2.3 - vbat)*0.9 - 0.158;
      required_compensation = (2.3 - vbat)*0.77 - 0.0916;
    }*/
    double x = (2.3 - vbat);
    double required_compensation = -2.8038*pow(x,6)+12.78*pow(x,5)-22.962*pow(x,4)+20.392*pow(x,3)-9.0565*pow(x,2)+2.4746*x-0.1464;
    
    
#endif
    if(!eStop)
    {
#if COMPENSATE
      double motor_pwm = throttle + (throttle-1000)*required_compensation;
      motor_pwm = constrain(motor_pwm, 1000, 2000);
      ledcWrite(ledChannel, MS_TO_PWM(motor_pwm));
      ledcWrite(ledChannelC, MS_TO_PWM(motor_pwm));
      ledcWrite(ledChannelD, MS_TO_PWM(motor_pwm));
#else
      ledcWrite(ledChannel, MS_TO_PWM(throttle));
      ledcWrite(ledChannelC, MS_TO_PWM(throttle));
      ledcWrite(ledChannelD, MS_TO_PWM(throttle));
#endif
    }

    presc++;
    update_orientation = 0;
  }
}
