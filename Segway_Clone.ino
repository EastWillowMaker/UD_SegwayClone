#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "MotorControl.h"
#include "rawData.h"
#include <avr/wdt.h>

float angle = 0.00;
int offset = 0;
unsigned int MotorTimeDoamin = 0;
void setup() {
  wdt_enable(WDTO_250MS);
  Serial.begin(115200);
  Wire.begin();
  main_set();
  motorInitial();
}

void loop() {
  //eastWillow degree update delay 2014/04/16
  int i = 0;
  for (i = 1; i <= 10; i++) {
    sample_angle();
    use_CompAngle();
    angle = (compAngleY - 180 - (offset)) / 100;
    timer = micros();
    wdt_reset();
  }
  getInitialAngle(angle);
  motorUpdateSpeed(angle);
}
