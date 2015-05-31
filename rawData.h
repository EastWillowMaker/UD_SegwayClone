
/* IMU Data */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXrate, gyroYrate;
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter

/*Adxl345*/
ADXL345 accel;

int16_t ax, ay, az;
/*Adxl345*/
/*L3G4200D*/
L3G4200D gyro;

int16_t avx, avy, avz;
/*L3G4200D*/
uint32_t timer;

void use_CompAngle() {
  compAngleX = (0.98 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.02 * accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.98 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.02 * accYangle);
}

void sample_angle() {
  /*Adxl345*/
  accel.getAcceleration(&ax, &ay, &az);
  /*Adxl345*/
  /*L3G4200D*/
  gyro.getAngularVelocity(&avx, &avy, &avz);
  /*L3G4200D*/
  accX = ax;
  accY = ay;
  accZ = az;
  gyroX = avx;
  gyroY = avy;
  gyroZ = avz;

  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  gyroXrate = (double)gyroX / 131.0;
  gyroYrate = -((double)gyroY / 131.0);
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
}
void main_set() {
  /*Adxl345*/
  // initialize device
  //Serial.println("Initializing I2C devices...");
  accel.initialize();

  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  // configure LED for output
  /*Adxl345*/
  /*L3G4200D*/
  // initialize device
  //Serial.println("Initializing I2C devices...");
  gyro.initialize();

  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(gyro.testConnection() ? "L3G4200D connection successful" : "L3G4200D connection failed");
  gyro.setFullScale(2000);
  /*L3G4200D*/
  timer = micros();
}
