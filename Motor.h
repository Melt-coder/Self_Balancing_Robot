#ifndef MOTOR_H
#define MOTOR_H
 
#include <Arduino.h>
 
#define Left_motor   7
#define Right_motor  12
#define PWM_LEFT     5
#define PWM_RIGHT    6
#define Enable_Pin   8

const int deadzone =25; // The minimum PWM required for the motor to overcome static is a hardware fact independent of the controller
 
void PinEnable();
void Forward();
void Backward();
void Left();
void Right();
void Stop();
void Motor_Control(float Speed); // Converts speed to PWM+ direction, regardless of which controller is used
 
#endif