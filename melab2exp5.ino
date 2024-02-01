#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "KalmanFilter.h"

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;
uint32_t timer;
double gyroXangle, gyroYangle;  // Angle calculate using the gyro only
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter
double gyr[2] = { 0, 0 };
//////////////////////////////////////////////////////////////////////////////////////////
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, Setpoint=0.0;
double cumError, rateError;

double rollreq = 20;
////////////////////////////////PID constants/////////////////////////////////////////////
double kp = 0.1;
double ki = 30;
double kd = 0.09;
//////////////////////////////////////////////////////////////////////////////////////////
const int ain1 = 8;
const int ain2 = 10;
const int pwma = 5;

void mpu_setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  double roll = 0.0;
  double pitch = 0.0;
  if (az != 0)
    roll = atan2(ay, az) * RAD_TO_DEG;
  else
    roll = atan2(ay, az + 0.0001) * RAD_TO_DEG;
  if (sqrt(ay * ay + az * az) != 0.0)
    pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  else
    pitch = atan(-ax / 0.00001 + sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  kalmanX.setAngle(roll);  // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

double computePID(double inp) {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime) / 1;
  error = Setpoint - inp;
  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;

  double out = kp * error + ki * cumError + kd * rateError;

  lastError = error;
  previousTime = currentTime;
  Serial.print("\t");
  Serial.println(out);
  if(out<=-255)
     out=-255;
  else if(out>=255)
     out = 255;
  return out;
}

void setup() {
  Serial.begin(9600);

  mpu_setup();
  delay(1000);

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwma, OUTPUT);

  delay(500);
}

double get_rp() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double dt = (double)(micros() - timer) /1000000;  // Calculate delta time
  timer = micros();
  double roll = 0.0;
  double pitch = 0.0;
  if (az != 0)
    roll = atan2(ay, az) * RAD_TO_DEG;
  else
    roll = atan2(ay, az + 0.0001) * RAD_TO_DEG;
  if (sqrt(ay * ay + az * az) != 0.0)
    pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  else
    pitch = atan(-ax / 0.00001 + sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  Serial.println(roll);
  return roll;
  delay(2);
}

void loop() {
  double roll = get_rp();
  double p = 0.0;
  if (abs(Setpoint-roll) > 2)
    p = computePID(roll);
  else p = 0;

  //motor output p
  // Serial.println(p);

  if (p> 0) {
    analogWrite(pwma, abs(p));
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
    delay(500);
    analogWrite(pwma, 0);
  } else if (p <0) {
    analogWrite(pwma, abs(p));
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    delay(500);
    analogWrite(pwma, 0);
  } else {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, HIGH);
  }
  delay(100);
}