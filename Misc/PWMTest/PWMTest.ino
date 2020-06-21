// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 12
#define VBAT_ADC 27
#define IMU_INTERRUPT 25
#define RF_INTERRUPT 15

// setting PWM properties
#define FREQ 30000
#define RESOLUTION 8
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define DUTY_TO_PWM(x) ((float)x)*((float)MAX_PWM)/100.0

// Variables for serial messages synchronization
bool started = false;
String message = "";
double pwm = 50;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA); // negative torque
  ledcWrite(ledChannelA, DUTY_TO_PWM(pwm));

  ledcSetup(ledChannelB, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, ledChannelB); // negative torque
  ledcWrite(ledChannelB, DUTY_TO_PWM(pwm));

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC); // negative torque
  ledcWrite(ledChannelC, DUTY_TO_PWM(pwm));

  ledcSetup(ledChannelD, FREQ, RESOLUTION);
  ledcAttachPin(PWMD, ledChannelD); // negative torque
  ledcWrite(ledChannelD, DUTY_TO_PWM(pwm));
}

// the loop function runs over and over again forever
void loop() {
  // Read and parse from serial
  while(Serial.available())
  {
    char temp = char(Serial.read());
    if(temp == '=')
    {
      started = true;
    }
    else if(temp == '\n')
    {
      started = false;
      pwm = constrain(message.toDouble(), 0, 100);
      Serial.println(pwm);
      message = "";
    }
    else if(started)
    {
      message += temp;
    }
  }
  
  ledcWrite(ledChannelA, DUTY_TO_PWM(pwm));
  ledcWrite(ledChannelB, DUTY_TO_PWM(pwm));
  ledcWrite(ledChannelC, DUTY_TO_PWM(pwm));
  ledcWrite(ledChannelD, DUTY_TO_PWM(pwm));
  
}
