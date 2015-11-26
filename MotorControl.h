#define LED_PIN 13
#define m_1_EN  7 //　Digital pin 2; 馬達控制板通電角位 Enable 1
#define m1_PWM1 11 // Digital pin 11; PWM_OC2A; m1_PWM 1
#define m1_PWM2 3 //　Digital pin 3; PWM_OC2B; m1_PWM 2

#define m_2_EN 4 //　Digital pin 4; 馬達控制板通電角位 Enable 2
#define m2_PWM1 9 // Digital pin 9; PWM_OC1A; m2_PWM 1
#define m2_PWM2 10 // Digital pin 10; PWM_OC1B; m2_PWM 2

#define gainPot 0     //　Analog pin 0; (增益)角度範圍可變電阻角位 A
#define steeringPot 1 //　Analog pin 1; potentiometer for steering 轉向可變電阻角位 A
//volatile
float initialAngle = 0;
//eastWillow Update Steering in 2014/04/16
int initialSteering = 0;
//eastWillow Update Steering in 2014/04/16
//eastWillow Update Gain in 2014/04/16_afternoon
int customGain = 0;
float err = 0;
float lastErr = 0;

float kp = 100;
float ki = 200;
float kd = 100;
float humanGain = 6;

bool getInitialAngleFlag = 0;

void motorInitial(void) {
  pinMode(m_1_EN, OUTPUT);
  pinMode(m1_PWM1, OUTPUT);
  pinMode(m1_PWM2, OUTPUT);

  pinMode(m_2_EN, OUTPUT);
  pinMode(m2_PWM1, OUTPUT);
  pinMode(m2_PWM2, OUTPUT);
  TCCR2A = _BV(COM2A1) | _BV(WGM20) | _BV(COM2B1);
  TCCR2B = _BV(CS21);

  TCCR1A = _BV(COM1A1) | _BV(WGM10) | _BV(COM1B1);
  TCCR1B = _BV(CS11);
  digitalWrite(m_1_EN, LOW);
  digitalWrite(m_2_EN, LOW);
  pinMode(LED_PIN, OUTPUT);
}
void getInitialAngle(float yaw) {
  if (getInitialAngleFlag == 0) {
    initialAngle = yaw;
    initialSteering = map(analogRead(steeringPot), 0, 1023, 0, 255);
    customGain = map(analogRead(gainPot), 0, 1023, 0, 5);
    if (millis() > 1000 && customGain == 1) {
      getInitialAngleFlag = 1;
      digitalWrite(LED_PIN, HIGH);
    }
  }
}
void motorUpdateSpeed(float yaw) {
  int U = 0;
  float P = 0;
  float I = 0;
  float D = 0;
  int m1_speed = 0;
  int m2_speed = 0;
  //eastWillow Update Steering in 2014/04/16
  int steeringValue = 0;
  int errSteeringValue = 0;
  steeringValue = map(analogRead(steeringPot), 0, 1023, 0, 255);
  //eastWillow Update Steering in 2014/04/16

  customGain = map(analogRead(gainPot), 0, 1023, 0, 10);
  /*while (customGain == 0)
    map(analogRead(gainPot), 0, 1023, 0, 10);*/

  //eastWillow Update CutomGain in 2014/04/16
  err = yaw - initialAngle;
  //eastWillow float P gain in 20140416
  float kp = 300000 * (1 / ((abs(err)+1) * 100));
  //eastWillow Version Motor Moudle fix 2014/04/11
  P = err;
  I = I + err; // fix 2014/11/06
  D = err - lastErr;
  U = round(P * kp + I * ki + D * kd);
  lastErr = err;
  m1_speed = m2_speed = U;
  //eastWillow Update Steering in 2014/04/16
  errSteeringValue = (steeringValue - initialSteering) * 0.6;
  m1_speed += -errSteeringValue;
  m2_speed += errSteeringValue;
  //eastWillow Update Steering in 2014/04/16
  m1_speed = constrain(m1_speed, -253, 253);
  m2_speed = constrain(m2_speed, -253, 253);

  Serial.print("U\t");
  Serial.print(U);
  Serial.print("err\t");
  Serial.print(err);
  Serial.print("Pgain\t");
  Serial.println(kp);

  if (m1_speed > 0) {
    OCR2A = 0;
    OCR2B = m1_speed;
  }
  else if (m1_speed < 0) {
    OCR2B = 0;
    m1_speed = abs(m1_speed);
    OCR2A = m1_speed;
  }
  else {
    OCR2A = 0;
    OCR2B = 0;
  }
  // Motor 2
  if (m2_speed > 0) {
    OCR1A = 0;
    OCR1B = m2_speed;
  }
  else if (m2_speed < 0) {
    OCR1B = 0;
    m2_speed = abs(m2_speed);
    OCR1A = m2_speed;
  }
  else {
    OCR1A = 0;
    OCR1B = 0;
  }
}
