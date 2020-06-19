#define RXD2 9
#define TXD2 15

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
#define FREQ 50
#define RESOLUTION 12
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((float)FREQ)
#define MS_TO_PWM(x) ((float)x)*((float)MAX_PWM)/(((float)MAX_PERIOD)*1000.0f)

#define ledChannel ledChannelB

// Attitude sampling
#define ORIENTATION_SAMPLING 0.25f

// timer variables
volatile uint8_t update_orientation;
hw_timer_t * orientation_timer = NULL;

// Timer ISR
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
}

// Variables for serial messages synchronization
bool started = false;
String message = "";
double pwm = 1000;

// Battery voltage variable
double vbat = 0;

// Measured speed
double rpm = 0;
 
void setup(){
  Serial.begin(500000, SERIAL_8N1);
  Serial2.begin(500000, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  Serial.println("");
  
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

  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);
  timerAlarmEnable(orientation_timer);
}
 
void loop(){
  
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
      pwm = message.toDouble();
      message = "";
    }
    else if(started)
    {
      message += temp;
    }
  }

  // Battery lpf
  double temp_meas = analogRead(37)*(3.3/4095.0)*(1.0076) + 0.1729;
  vbat = 0.92*vbat + 0.08*temp_meas;

  // rpm lpf
  rpm = 0.92*rpm + 0.08*analogRead(ADCB)*33.0/4095.0;

  // Battery Compensation
  double x = (2.3 - vbat);
  double required_compensation = -2.8038*pow(x,6)+12.78*pow(x,5)-22.962*pow(x,4)+20.392*pow(x,3)-9.0565*pow(x,2)+2.4746*x-0.1464;

  // Write PWM
  double motor_pwm = pwm + (pwm-1000)*required_compensation;
  motor_pwm = constrain(motor_pwm, 1000, 2000);
  if(pwm <= 1150)
  {
    motor_pwm = 1000;
  }
  ledcWrite(ledChannel, MS_TO_PWM(motor_pwm));

  if(update_orientation)
  {

    // Log data
    Serial.print(vbat);
    Serial.print(" ");
    Serial.print(rpm);
    Serial.print(" ");
    Serial.println(pwm);
    update_orientation = 0;
  }
}
