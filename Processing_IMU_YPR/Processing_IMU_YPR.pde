// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-20 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

ToxiclibsSupport gfx;

Serial port;                         // The serial port
int interval = 0;

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

String whole = "";
int serialCount = 0; 
int synced = 0;

void setup() {
    // 300px square viewport using OpenGL rendering
    size(500, 500, OPENGL);
    gfx = new ToxiclibsSupport(this);

    // setup lights and antialiasing
    lights();
    smooth();
  
    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)
    //String portName = Serial.list()[0];
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    String portName = "COM42";
    
    // open the serial port
    //port = new Serial(this, portName, 115200);
    port = new Serial(this, portName, 1000000);
    
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)
    //port.write('r');
    delay(1000);
    
    //camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 0, 1);
    //camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
    //camera(200, 0, (height/2.0) / tan(PI*30.0 / 180.0), 250, 250, 0, 0, 0, -1);
    beginCamera();
    camera(0, 0, 0, 200, 200, 0, 0, 0, -1);
    rotateY(PI);
    translate(-400,-100,0);
    //translate(500, 500);
    scale(0.9);
    translate(-100,200,0);
    endCamera();
}

void draw() {
    
    // black background
    background(0);
    
    // translate everything to the middle of the viewport
    pushMatrix();
    translate(width / 2, height / 2);

    // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
    // ...and other weirdness I haven't figured out yet
    rotateZ(ypr[2]);
    rotateY(ypr[1]);
    rotateX(ypr[0]);
    /*
    rotateX(ypr[0]);
    rotateY(ypr[2]);
    rotateZ(ypr[1]);*/

    // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    // different coordinate system orientation assumptions between Processing
    // and InvenSense DMP)
    //float[] axis = quat.toAxisAngle();
    //rotate(axis[0], -axis[1], axis[3], axis[2]);
    
    //print axis-X
    stroke(255, 0, 0);
    line(-300, 0, 0, 200, 0, 0);
    
    //print axis-y
    stroke(0, 255, 0);
    line(0, -200, 0, 0, 300, 0);
    
    //print axis-z
    stroke(0, 0, 255);
    line(0, 0, -200, 0, 0, 200);
    
    //print axis-z
    /*stroke(0, 0, 255);
    strokeWeight(2);
    line (0, 0, 300, 25, 0, 275);
    line (0, 0, 300, -25, 0, 275); 
    line (0, 20, 320, 0, 0, 330);
    line (0, 0, 330, 0, 20, 340);
    line (0, 0, 330, 0, -20, 330);
    line(0, 0, -300, 0, 0, 300);*/

    // draw main body in red
    fill(255, 0, 0, 200);
    box(10, 10, 200);
    
    // draw front-facing tip in blue
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();
    
    // draw wings and tail fin in green
    fill(0, 255, 0, 200);
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  // wing top layer
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  // wing bottom layer
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  // tail left layer
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  // tail right layer
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();
    
    popMatrix();
}

void serialEvent(Serial port) {
  while(port.available() > 0)
  {
      int ch = port.read();
      
      //String inString = port.readString();
      if(ch != 'y' && synced == 0) return;
      synced = 1;
      
      if(serialCount > 100 && ch != '$')
      {
        serialCount = 0;
        synced = 0;
        return;
      }
      
      if(serialCount > 0 || ch == 'y')
      {
          serialCount++;
          whole += (char)ch;
          if(ch == '$')
          {
              String vals[] = split(whole, ',');
              ypr[0] = float(trim(vals[0].replace('y', ' ')));
              ypr[1] = float(trim(vals[1]));
              ypr[2] = float(trim(vals[2].replace('$', ' ')));
              //ypr[2] = 0;
              println(whole);
              whole = "";
              serialCount = 0;
          }
      }
  }
  
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
    
        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}
