#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "Mahony.h"
#include "lowpassfilter.h"
#include "PID.h"
#include "MatrixOperator.h"
#define FILTER_N 12
#define sample_rate 100.0f
#define cutoff_frequency 10.0f
#define kp 0.20f
#define ki 0.015f
#define kd 0.20f

const int MPU = 0x68;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float gX[FILTER_N + 1], gY[FILTER_N + 1], gZ[FILTER_N + 1];
float magX, magY, magZ;
float pitch, yaw, roll;
int c ;
float expectation = 1.0f;
float motor1, motor2, motor3, motor4;
float tx, ty, tz;

Mahony maho;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
struct _PID inpidX;
struct _PID inpidY;
struct _PID inpidZ;
struct _PID outpidX;
struct _PID outpidY;
struct _PID outpidZ;

float filter_aver(float filter_buf[FILTER_N + 1]) {
  int filterSum = 0;
  for (int i = 0; i < FILTER_N; i++)
  {
    filter_buf[i] = filter_buf[i + 1];
    filterSum += filter_buf[i];
  }
  return (float)(filterSum / FILTER_N);
}

float curve(float input)
{
  float output;
  if (input <= 0.125) {
    output = 0.356 * input;
  }
  if (input > 0.125 & input <= 0.250) {
    output = 0.0445 + 0.456 * input;
  }
  if (input > 0.250 & input <= 0.375) {
    output = 0.1015 + 0.5864 * input;
  }
  if (input > 0.375 & input <= 0.50) {
    output = 0.1748 + 0.7528 * input;
  }
  if (input > 0.50 & input <= 0.625) {
    output = 0.2689 + 0.9672 * input;
  }
  if (input > 0.625 & input <= 0.750) {
    output = 0.3898 + 1.2408 * input;
  }
  if (input > 0.750 & input <= 0.875) {
    output = 0.5449 + 1.5944 * input;
  }
  if (input > 0.875 & input <= 1) {
    output = 0.7442 + 2.0464 * input;
  }
  return output;
}

void allot(int n, float x[], float y[])
{
  float ma[4][n];
  for (int j = 0; j < n; j++) {
    ma[0][j] = 1;
  }
  for (int j = 0; j < n; j++) {
    ma[1][j] = -y[j];
  }
  for (int j = 0; j < n; j++) {
    ma[2][j] = x[j];
  }
  for (int j = 0; j < n; j++) {
    ma[3][j] = (-1) ^ (j);
  }
  Matrix mA;
  mA.Init((float *)ma, 4, n);
  Matrix pinv_mA = MatrixOperator::pinv(mA);
}

void setup(void) {

  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  PID_init(&inpidX, kp, ki, kd);
  PID_init(&inpidY, kp, ki, kd);
  PID_init(&inpidZ, kp, ki, kd);
  PID_init(&outpidX, kp, 0, 0);
  PID_init(&outpidY, kp, 0, 0);
  PID_init(&outpidZ, kp, 0, 0);
}
unsigned long starttime;
unsigned long stoptime;
unsigned long remaintime;
void loop()
{
  starttime = millis();
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);              // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  accX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  accY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  accZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  lowpassfilter filterx(sample_rate, cutoff_frequency);
  lowpassfilter filtery(sample_rate, cutoff_frequency);
  lowpassfilter filterz(sample_rate, cutoff_frequency);

  accX = filterx.Update(accX) - 0.01f;
  accY = filtery.Update(accY) + 0.01f;
  accZ = filterz.Update(accZ) - 0.43f;

  // === Read gyroscope data === //

  if (c < FILTER_N )
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);                 // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);  // Read 4 registers total, each axis value is stored in 2 registers
    gX [c] = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    gY [c] = (Wire.read() << 8 | Wire.read()) / 131.0;
    gZ [c] = (Wire.read() << 8 | Wire.read()) / 131.0;
  }
  else
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gX [FILTER_N ] = (Wire.read() << 8 | Wire.read()) / 131.0;
    gY [FILTER_N ] = (Wire.read() << 8 | Wire.read()) / 131.0;
    gZ [FILTER_N ] = (Wire.read() << 8 | Wire.read()) / 131.0;

    gyroX = filter_aver(gX) - 3.0f;
    gyroY = filter_aver(gY) - 2.0f;
    gyroZ = filter_aver(gZ) - 2.0f;

    // === Read magnetic data === //
    //    sensors_event_t event;
    //    mag.getEvent(&event);
    //
    //    magX = event.magnetic.x;
    //    magY = event.magnetic.y;
    //    magZ = event.magnetic.z;

    //    Serial.print("X: "); Serial.print(magX); Serial.print("  ");
    //    Serial.print("Y: "); Serial.print(magY); Serial.print("  ");
    //    Serial.print("Z: "); Serial.print(magZ); Serial.print("  "); Serial.println("uT");

    //    maho.upDate( gyroX, gyroY, gyroZ, accX, accY, accZ, magX, magY, magZ);
    maho.upDate( gyroX, gyroY, gyroZ, accX, accY, accZ, 0.0f, 0.0f, 0.0f);

    pitch = maho.getpitch();
    yaw = maho.getyaw();
    roll = maho.getroll();

    outpidX.measurement = PID_update(&outpidX, yaw, expectation);
    outpidY.measurement = PID_update(&outpidY, roll, expectation);
    outpidZ.measurement = PID_update(&outpidZ, pitch, expectation);
    inpidX.measurement = PID_update(&inpidX, gyroX, outpidX.measurement);
    inpidY.measurement = PID_update(&inpidY, gyroY, outpidY.measurement);
    inpidZ.measurement = PID_update(&inpidZ, gyroZ, outpidZ.measurement);

    float L = 20.0f;
    tx = 1.0f * inpidX.measurement;
    ty = 1.0f * inpidY.measurement;
    tz = 1.0f * inpidZ.measurement;
    //+四旋翼
    //    motor1 = 0.25 * (L + 2 * ty + tz);
    //    motor2 = 0.25 * (L - 2 * tx - tz);
    //    motor3 = 0.25 * (L - 2 * ty + tz);
    //    motor4 = 0.25 * (L + 2 * tx - tz);

    //x四旋翼
    motor1 = 0.25 * (L + 1.41421356 * tx + 1.41421356 * ty + tz);
    motor2 = 0.25 * (L - 1.41421356 * tx + 1.41421356 * ty - tz);
    motor3 = 0.25 * (L - 1.41421356 * tx - 1.41421356 * ty + tz);
    motor4 = 0.25 * (L + 1.41421356 * tx - 1.41421356 * ty - tz);

    //六旋翼
    //    motor1 = 0.166667 * (L + 2 * ty + tz);
    //    motor2 = 0.166667 * (L - 1.7320508 * tx + ty + tz);
    //    motor3 = 0.166667 * (L - 1.7320508 * tx - ty + tz);
    //    motor4 = 0.166667 * (L - 2 * ty - tz);
    //    motor5 = 0.166667 * (L + 1.7320508 * tx - ty + tz);
    //    motor6 = 0.166667 * (L + 1.7320507 * tx + ty - tz);

    if (readyToPrint()) {

      //      Serial.print("pitch: ");
      //      Serial.print(pitch);
      //      Serial.print(",");
      //      Serial.print("yaw:  ");
      //      Serial.print(yaw);
      //      Serial.print(",");
      //      Serial.print("roll:  ");
      //      Serial.println(roll);
    }
  }

  c++;

  //  frequency=100Hz,T=10ms
  stoptime = millis();
  remaintime = 10 - (stoptime - starttime);
  if (remaintime > 0)
  {
    delay(remaintime);
  }
}
// Decide when to print
bool readyToPrint()
{
  static unsigned long nowMillis;
  static unsigned long thenMillis;

  // If the Processing visualization sketch is sending "s"
  // then send new data each time it wants to redraw
  while (Serial.available()) {
    int val = Serial.read();
    if (val == 's') {
      thenMillis = millis();
      return true;
    }
  }
  // Otherwise, print 10 times per second, for viewing as
  // scrolling numbers in the Arduino Serial Monitor
  nowMillis = millis();
  if (nowMillis - thenMillis > 100) {
    thenMillis = nowMillis;
    return true;
  }
  return false;
}
