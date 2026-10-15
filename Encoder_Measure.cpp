#include "Encoder.h"
 
volatile long Encoder_Count_Left  = 0;
volatile long Encoder_Count_Right = 0;

// For the T-method: the time of the last pulse and the time interval between two pulses(microseconds)
volatile unsigned long Encoder_LastPulseTime_Left  = 0;
volatile unsigned long Encoder_LastPulseTime_Right = 0;
volatile unsigned long Encoder_PulseInterval_Left  = 0;
volatile unsigned long Encoder_PulseInterval_Right = 0;
 
void Encoder_Pin_Enable() {
  pinMode(ENCODER_PIN_LEFT,  INPUT_PULLUP);
  pinMode(ENCODER_PIN_RIGHT, INPUT_PULLUP);
  // The direction pins will be read, so they must also be accessible as INPUTs
  pinMode(Enable_Pin, OUTPUT);
  digitalWrite(Enable_Pin, HIGH);  // Standby closed
}
 
// The counter is increased or decreased by reading the direction pin
void ISR_LEFT() {
  // T-method: Record the time elapsed between this pulse and the previos one
  // This will be used in the main loop to calculate "time between pulses=1/speed"
  unsigned long now = micros();
  Encoder_PulseInterval_Left = now - Encoder_LastPulseTime_Left;
  Encoder_LastPulseTime_Left = now;

  // If the motor is moving forward, AIN1(DIR_PIN_LEFT) goes LOW(according to Forward function in Motor.cpp)
  if (digitalRead(DIR_PIN_LEFT) == LOW) {
    Encoder_Count_Left++;
  } else {
    Encoder_Count_Left--;
  }
}
 
void ISR_RIGHT() {
 unsigned long now = micros();
  Encoder_PulseInterval_Right = now - Encoder_LastPulseTime_Right;
  Encoder_LastPulseTime_Right = now;

  
  if (digitalRead(DIR_PIN_RIGHT) == LOW) {
    Encoder_Count_Right++;
  } else {
    Encoder_Count_Right--;
  }
}