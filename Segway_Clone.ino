#include "I2Cdev.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include <Wire.h>
#include "MotorControl.h"
#include "rawData.h"
//#include <avr/wdt.h>

float angle = 0.00;
unsigned int MotorTimeDoamin = 0;
void setup() {
  //wdt_enable(WDTO_250MS);
  Serial.begin(115200);
  Serial.setTimeout(10);
  Wire.begin();
  main_set();
  motorInitial();
}

void loop() {
  //eastWillow degree update delay 2014/04/16
  //int i = 0;
  //static float avgAngle = 0;
  // Filter Start
  //for(i = 0 ; i< 25;i++){
    sample_angle();
    use_CompAngle();
    angle = -(compAngleX - 180);
    timer = micros();
    //if(angle >= -0.15 && angle <= 0.15){
      //angle = 0;
  //}
    //avgAngle += angle;
    //getInitialAngle(angle);
  //}
  //avgAngle /= 25;
  //angle = -(compAngleX - 180);
  //wdt_reset();
  getInitialAngle(angle);
  motorUpdateSpeed(angle);
}
