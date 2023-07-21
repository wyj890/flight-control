#include "Mahony.h"
#include <math.h>
#define  halfDt 0.005f
#define  Kp 0.8f
#define  Ki 0.0005f

Mahony::Mahony() {
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  intex = 0.0f;
  intey = 0.0f;
  intez = 0.0f;
}

void Mahony::upDate(float gx, float gy, float gz, float ax, float ay, float az, float mx , float my , float mz )
{
  float invnorm;
  float vx, vy, vz;
  float wx, wy, wz;
  float hx, hy, hz;
  float bx, bz;
  float ex, ey, ez;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float ntob11, ntob12, ntob13, ntob21, ntob22, ntob23, ntob31, ntob32, ntob33;
  float errorGx, errorGy, errorGz;
  float q00, q11, q22;

  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  ntob11 = q0q0 + q1q1 - q2q2 - q3q3;
  ntob12 = 2 * (q1q2 + q0q3);
  ntob13 = 2 * (q1q3 - q0q2);
  ntob21 = 2 * (q1q2 - q0q3);
  ntob22 = q0q0 - q1q1 + q2q2 - q3q3;
  ntob23 = 2 * (q2q3 + q0q1);
  ntob31 = 2 * (q1q3 + q0q2);
  ntob32 = 2 * (q2q3 + q0q1);
  ntob33 = q0q0 - q1q1 - q2q2 + q3q3;

  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  if ((mx == 0) & (my == 0) & (mz == 0))
  {
    invnorm = 1 / sqrt(ax * ax + ay * ay + az * az);
    ax = ax * invnorm;
    ay = ay * invnorm;
    az = az * invnorm;

    vx = ntob13;
    vy = ntob23;
    vz = ntob33;

    ex = (ay * vz - az * vy) ;
    ey = (az * vx - ax * vz) ;
    ez = (ax * vy - ay * vx) ;
    intex += Ki * ex ;
    intey += Ki * ey ;
    intez += Ki * ez ;
    errorGx = Kp * ex + intex;
    errorGy = Kp * ey + intey;
    errorGz = Kp * ez + intez;
    gx += errorGx;
    gy += errorGy;
    gz += errorGz;

    q00 = q0;
    q11 = q1;
    q22 = q2;

    q0 +=  halfDt * (-gx * q11 - gy * q22 - gz * q3);
    q1 += halfDt * (gx * q00 - gy * q3 + q22 * gz);
    q2 += halfDt * (q3 * gx + q00 * gy - q11 * gz);
    q3 += halfDt * (-q22 * gx + q11 * gy + q00 * gz);

    invnorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * invnorm;
    q1 = q1 * invnorm;
    q2 = q2 * invnorm;
    q3 = q3 * invnorm;

    roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    pitch = asinf(2.0f * (-q1 * q3 + q0 * q2));
    yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

  }

  else
  {
    invnorm = 1 / sqrt(ax * ax + ay * ay + az * az);
    ax = ax * invnorm;
    ay = ay * invnorm;
    az = az * invnorm;
    invnorm = 1 / sqrt(mx * mx + my * my + mz * mz);
    mx = mx * invnorm;
    my = my * invnorm;
    mz = mz * invnorm;

    vx = ntob13;
    vy = ntob23;
    vz = ntob33;

    hx = mx * ntob11 + my * ntob21 + mz * ntob31;
    hy = mx * ntob12 + my * ntob22 + mz * ntob32;
    hz = mx * ntob13 + my * ntob23 + mz * ntob33;

    bx = sqrtf(hx * hx + hy * hy);
    bz = hz;
    wx = bx * ntob11 + bz * ntob31;
    wy = bx * ntob12 + bz * ntob32;
    wz = bx * ntob13 + bz * ntob33;

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    intex += Ki * ex ;
    intey += Ki * ey ;
    intez += Ki * ez ;
    errorGx = Kp * ex + intex;
    errorGy = Kp * ey + intey;
    errorGz = Kp * ez + intez;
    gx += errorGx;
    gy += errorGy;
    gz += errorGz;

    q00 = q0;
    q11 = q1;
    q22 = q2;

    q0 = q00 + halfDt * (-gx * q11 - gy * q22 - gz * q3);
    q1 = q11 + halfDt * (gx * q00 - gy * q3 + q22 * gz);
    q2 = q22 + halfDt * (q3 * gx + q00 * gy - q11 * gz);
    q3 = q3 + halfDt * (-q22 * gx + q11 * gy + q00 * gz);

    invnorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * invnorm;
    q1 = q1 * invnorm;
    q2 = q2 * invnorm;
    q3 = q3 * invnorm;

    roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
    yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);


  }
}

float Mahony::getpitch()
{
  return pitch * 57.2957795f;
}

float Mahony::getyaw()
{
  return yaw * 57.2957795f;
}

float Mahony::getroll()
{
  return roll * 57.2957795f;
}
