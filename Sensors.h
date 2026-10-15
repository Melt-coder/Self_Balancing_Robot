#ifndef SENSORS_H
#define SENSORS_H
 
#include <Arduino.h>
#include <MPU6050_tockn.h>
 
extern MPU6050 mpu6050;
 
class Sensors {
public:
  float Angle_MPU();    // Returns the filtered angle (for P and I)
  float Gyro_MPU();    //  Returns the angular velocity degrees/sec (for D)
  void  Init_MPU();
  void  Print_MPU();
 
  float Angle_X = 0.0;
  

  float FilteredAngle= 0.0; // Used for derivate term for a low-pass filter
  float GyroRate = 0.0;     // Raw gyro output

   // State varibles required my own complementary filter
  unsigned long lastAngleUpdate = 0; // the time of previous Angle_MPU() call (for dt)
  bool angleInitialized = false;     // Initializing FilteredAngle from the accelerometer on first call
};
 
#endif