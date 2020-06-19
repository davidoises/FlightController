#define PWMA 32
#define READING_PIN 12

// setting PWM properties
#define FREQ 10
#define RESOLUTION 8
#define ledChannelA 0

#define MAX_PWM (1<<RESOLUTION)-1
#define DUTY_TO_PWM(x) ((float)x)*((double)MAX_PWM)

float on_time = 0;//pulseIn(oin, HIGH)
float off_time = 0; //pulseIn(oin, LOW)
float period = 0;
float freq = 0;
float duty = 0;

unsigned long prev_time = 0;

TaskHandle_t measurement_handle = NULL;

volatile uint8_t update_measurements;
void IRAM_ATTR reading_isr() {
  update_measurements = 1;
}

void measurementLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {
    if(update_measurements)
    {
      unsigned long current_time = micros();
      period = current_time - prev_time;
      prev_time = current_time;

      freq = 1000000.0/period;
     
      update_measurements = 0;
    }
    /*on_time = pulseIn(READING_PIN, HIGH);
    off_time = pulseIn(READING_PIN, LOW);
    period = on_time + off_time;
    freq = 1.0/period;
    duty = on_time/period;*/
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting");

  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA); // negative torque
  ledcWrite(ledChannelA, DUTY_TO_PWM(0.5));
  
  xTaskCreatePinnedToCore(
    measurementLoop,      /* Function to implement the task */
    "Measurement core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &measurement_handle,           /* Task handle. */
    0);

   attachInterrupt(READING_PIN, reading_isr, RISING);
   prev_time = micros();
}

void loop() {
  Serial.println(freq);
  vTaskDelay(100);
}
