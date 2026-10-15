#include "Sensors.h"
#include <Wire.h>
 
MPU6050 mpu6050(Wire);
 
void Sensors::Init_MPU() {
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}
 
float Sensors::Angle_MPU() {
  mpu6050.update();
  
  // Calculating angle from accelerometer
  float accY = mpu6050.getAccY();
  float accZ = mpu6050.getAccZ();
  float accelAngle = atan2(accY, accZ) * 180.0 / PI;
  Angle_X = accelAngle; // raw accelerometer angle

  //  Time difference(dt)
  unsigned long now = millis();
  float dt = (now - lastAngleUpdate) / 1000.0;
  lastAngleUpdate = now;
  if (dt <= 0.0 || dt > 0.5) dt = 0.01; // Safe assumption for the initial call or abnormal jump

  float gyroRateNow = mpu6050.getGyroX(); // angle/sec

  // Complementary filter
  if (!angleInitialized) {
    FilteredAngle = accelAngle;
    angleInitialized = true;
  } else {
    const float alpha = 0.99; // 0.99: tau about 0.5sec -> about1sec 
    FilteredAngle = alpha * (FilteredAngle + gyroRateNow * dt) + (1.0 - alpha) * accelAngle;
  }

  return FilteredAngle;
}

float Sensors::Gyro_MPU() {
  GyroRate = mpu6050.getGyroX();
  return GyroRate;
}
 
void Sensors::Print_MPU() {
  Angle_MPU();
  Serial.print("Angle_X: ");
  Serial.println(Angle_X);
  Serial.print(" GyroRate: ");
  Serial.println(GyroRate);
}