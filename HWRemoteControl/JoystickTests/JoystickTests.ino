float jx = 1500;
float jy = 1500;

float mapJoystickValues(int val, int middle_point, float lower_output,  float upper_output)
{
  float output_middle_point = (lower_output + upper_output)/2;
  float temp = 0;
  if(val < middle_point)
  {
    temp = map(val, 0, middle_point, lower_output, output_middle_point);
  }
  else
  {
    temp = map(val, middle_point, 4095, output_middle_point, upper_output);
  }
  return temp;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  float x = mapJoystickValues(analogRead(14), 1907, 1000.0, 2000.0);
  float y = mapJoystickValues(analogRead(12), 1843, 1000.0, 2000.0);

  jx = 0.7*jx + 0.3*x;
  jy = 0.7*jy + 0.3*y;
  
  // put your main code here, to run repeatedly:
  Serial.print(jx); //X
  Serial.print(" ");
  Serial.println(jy); //Y
  delay(40);
}
