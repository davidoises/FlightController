#include "PsxLib.h"

Psx ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

void setup(){
 
  Serial.begin(115200);
  
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it
   
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  //error = ps2x.config_gamepad(15, 4, 14, 12, true, true);
  ps2x.setupPins(26, 27, 25, 32, 50);
}

void loop() {
  
  unsigned long datos = ps2x.read();
  
  if(ps2x.button(PSB_START))         //will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  if(ps2x.button(PSB_SELECT))
    Serial.println("Select is being held");      

  if(ps2x.button(PSB_PAD_UP))       //will be TRUE as long as button is pressed
    Serial.println("Up held this hard: ");
  if(ps2x.button(PSB_PAD_RIGHT))
    Serial.println("Right held this hard: ");
  if(ps2x.button(PSB_PAD_LEFT))
    Serial.println("LEFT held this hard: ");
  if(ps2x.button(PSB_PAD_DOWN))
    Serial.println("DOWN held this hard: ");

  if(ps2x.button(PSB_TRIANGLE))
    Serial.println("Triangle pressed");
  if(ps2x.button(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    Serial.println("Circle just pressed");
  if(ps2x.button(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
    Serial.println("X just changed");
  if(ps2x.button(PSB_SQUARE))              //will be TRUE if button was JUST released
    Serial.println("Square just released");

  if(ps2x.button(PSB_L3))
    Serial.println("L3 pressed");
  if(ps2x.button(PSB_R3))
    Serial.println("R3 pressed");
  if(ps2x.button(PSB_L2))
    Serial.println("L2 pressed");
  if(ps2x.button(PSB_R2))
    Serial.println("R2 pressed");

  if(ps2x.button(PSB_L1) || ps2x.button(PSB_R1)) { //print stick values if either is TRUE
    Serial.print(ps2x.analog(0), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
    Serial.print(",");
    Serial.print(ps2x.analog(1), DEC); 
    Serial.print(",");
    Serial.print(ps2x.analog(2), DEC); 
    Serial.print(",");
    Serial.println(ps2x.analog(3), DEC);
  }
  delay(50);  
}
