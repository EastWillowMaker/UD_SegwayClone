#define LED_PIN 13
#define m_1_EN  7 //　Digital pin 2; 馬達控制板通電角位 Enable 1/red
#define m1_PWM1 3 // Digital pin 11; PWM_OC2A; m1_PWM 1
#define m1_PWM2 11 //　Digital pin 3; PWM_OC2B; m1_PWM 2

#define m_2_EN 4 //　Digital pin 4; 馬達控制板通電角位 Enable 2
#define m2_PWM1 10 // Digital pin 9; PWM_OC1A; m2_PWM 1
#define m2_PWM2 9 // Digital pin 10; PWM_OC1B; m2_PWM 2

#define steeringPot A0 //　Analog pin 1; potentiometer for steering 轉向可變電阻角位 A
#define Firstkp 6
#define Secondkp 8
#define Thirdkp 9
#define Fourthkp 11
//volatile
float initialAngle = 0;
//eastWillow Update Steering in 2014/04/16
int initialSteering = 0;
//eastWillow Update Steering in 2014/04/16
//eastWillow Update Gain in 2014/04/16_afternoon
//eastWilow remove custom Gain 2014/04/18
float err = 0;
float lastErr = 0;
float totalErr = 0;
unsigned long errCounter = 0;

float kp = Firstkp;
float ki = 0;
float kd = 0;
float N = 1;

int m1_offset = 0;
//20150514 eastWillow Fix PID

//eastWillow Frequency Cancel in 2015/02/23
unsigned long postiveTime = 0;
unsigned long negativeTime = 0;
unsigned long zeroTime = 0;

bool getInitialAngleFlag = 0;

void motorInitial(void) {
  pinMode(m_1_EN, OUTPUT);
  pinMode(m1_PWM1, OUTPUT);
  pinMode(m1_PWM2, OUTPUT);

  pinMode(m_2_EN, OUTPUT);
  pinMode(m2_PWM1, OUTPUT);
  pinMode(m2_PWM2, OUTPUT);
  /*TCCR2A = _BV(COM2A1) | _BV(WGM20) | _BV(COM2B1);
  TCCR2B = _BV(CS21);

  TCCR1A = _BV(COM1A1) | _BV(WGM10) | _BV(COM1B1);
  TCCR1B = _BV(CS11);*/
  digitalWrite(m_1_EN, LOW);
  digitalWrite(m_2_EN, LOW);
  pinMode(LED_PIN, OUTPUT);
}
void getInitialAngle(float yaw) {
  if (getInitialAngleFlag == 0) {
    initialAngle = yaw;
    initialSteering = analogRead(steeringPot);
    if (millis() > 3000) {
      getInitialAngleFlag = 1;
      digitalWrite(LED_PIN, HIGH);
    }
  }
}
void motorUpdateSpeed(float yaw) {
  //eastWillow 20150522
  static int U = 0;
  static float P = 0;
  static float I = 0;
  static float FilterCoefficient = 0;
  static float Filter_DSTATE = 0;
  static int m1_speed = 0;
  static int m2_speed = 0;
  int counter = 0;
  //eastWillow Update Steering in 2014/04/16
  int steeringValue = 0;
  static int errSteeringValue = 0;
  steeringValue = analogRead(steeringPot);
  //eastWillow Update Steering in 2014/04/16
  err = yaw - initialAngle;
  //eastWillow errTriger in 20150523
  if(err >= -0.2 && err <= 0.2){
    err = 0;
    I = 0;
    digitalWrite(m_1_EN, LOW);
    digitalWrite(m_2_EN, LOW);
  }
  if(err <= -0.2){
    err -= -0.2;
    digitalWrite(m_1_EN, LOW);
    digitalWrite(m_2_EN, LOW);
  }
  if(err >= 0.2){
    err -= 0.2;
    digitalWrite(m_1_EN, LOW);
    digitalWrite(m_2_EN, LOW);
  }
  //eastWillow Frequency Cancel in 2015/02/23
  /*if(err > 2){
    postiveTime = millis();
  }
  if(err == 0){
    zeroTime = millis();
  } 
  if(err < -2){
    negativeTime = millis();
  }
  if(abs(postiveTime - zeroTime) < 30 || abs(zeroTime - negativeTime) < 30){
    err = 0;
  }
  */
  //eastWillow errTriger in 20150523
  //eastWillow float P gain in 20140416
  //float kp = 300000 * (1 / ((abs(err)+1) * 100));
  //eastWillow Version Motor Moudle fix 2014/04/11
  P = err;
  //eastWillow vibration P gain 20150606
  if(abs(err) > 5){
   kp = Fourthkp;
  }
  else if(abs(err) > 3){
   kp = Thirdkp;
  }
 else if(abs(err) > 2){
  kp = Secondkp;
 }
 else{
   kp = Firstkp;
 }
  I = I + err; // fix 2014/11/06
  FilterCoefficient = (kd * err - Filter_DSTATE) * N;
  //eastWillow D error protect
  /*if(D * kd > 40 || D * kd < -40){
    D = 0;
  }*/
  U = round(P * kp + I * ki + FilterCoefficient);
  Filter_DSTATE += FilterCoefficient;
  //eastWillow 20150523 get simu data , simu output
  /*if(getInitialAngleFlag == 1){
  if(millis() % 1000 <= 300){
    U = 30;
  }
  else if((millis() % 1000) <= 600){
    U = 0;
  }
  else if((millis() % 1000) <= 900){
    U = -30;
  }
  }*/
  //eastWillow 20150606 err Serial Output
  /*Serial.print(millis());
  Serial.print("\t");
  Serial.print(U);
  Serial.print("\t");
  Serial.print(errSteeringValue);
  Serial.print("\t");
  Serial.print(err);
  Serial.println();*/
  lastErr = err;
  m1_speed = U;
  m2_speed = U;
  //Serial.println(U);
  //eastWillow Update Steering in 2014/04/16
  errSteeringValue = steeringValue - initialSteering;
  errSteeringValue = constrain(errSteeringValue, -400, 400);
  errSteeringValue = map(errSteeringValue, -400, 400, -60, 60);
  //eastWillow Update Steering in 2014/04/16
  m1_speed = constrain(m1_speed+errSteeringValue, -230, 230);
  m2_speed = constrain(m2_speed-errSteeringValue, -230, 230);
  
  /* 20160201 eastwillow try to collect data */
  while(Serial.available() > 0){
    int inputTestData = 0;
    inputTestData = Serial.parseInt();
    m1_Speed = inputTestData;
    m2_Seeed = inputTestData
  }
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(err);
  Serial.println();
  if (m1_speed > 0) {
    analogWrite(m1_PWM1, 0);
    analogWrite(m1_PWM2, m1_speed+10);
  }
  else if (m1_speed < 0) {
    analogWrite(m1_PWM2, 0);
    analogWrite(m1_PWM1, abs(m1_speed));
  }
  else {
    analogWrite(m1_PWM1, 0);
    analogWrite(m1_PWM2, 0);
  }
  // Motor 2
  if (m2_speed > 0) {
    analogWrite(m2_PWM1, 0);
    analogWrite(m2_PWM2, m2_speed+10);
  }
  else if (m2_speed < 0) {
    analogWrite(m2_PWM2, 0);
    analogWrite(m2_PWM1, abs(m2_speed));
  }
  else {
    analogWrite(m2_PWM1, 0);
    analogWrite(m2_PWM2, 0);
  }

}
