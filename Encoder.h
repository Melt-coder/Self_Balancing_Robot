#ifndef ENCODER_H
#define ENCODER_H
 
#include <Arduino.h>
 
// Encoder A Phase pins
#define ENCODER_PIN_LEFT   2
#define ENCODER_PIN_RIGHT  4
#define Enable_Pin         8  // STBY
 
// Motor direction pins- Required for reading the direction within the ISR
// Alse defined here for independent access outside of Motor.h
#define DIR_PIN_LEFT   7   // Left_motor (AIN1)
#define DIR_PIN_RIGHT  12  // Right_motor (BIN1)
 
extern volatile long Encoder_Count_Left;
extern volatile long Encoder_Count_Right;

// For the T-method (measuring the time between pulses): ISR updates the these values at each pulse. 
// Encoder_Count_Left/Right is still used for position- this is just additional information to improve SPEED estimate; it does not change the position logic
extern volatile unsigned long Encoder_LastPulseTime_Left;
extern volatile unsigned long Encoder_LastPulseTime_Right;
extern volatile unsigned long Encoder_PulseInterval_Left;
extern volatile unsigned long Encoder_PulseInterval_Right;
 
void Encoder_Pin_Enable();
void ISR_LEFT();
void ISR_RIGHT();
 
#endif