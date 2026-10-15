 #include "Motor.h"
 
void PinEnable() {
  pinMode(Left_motor,  OUTPUT);
  pinMode(Right_motor, OUTPUT);
  pinMode(PWM_LEFT,    OUTPUT);
  pinMode(PWM_RIGHT,   OUTPUT);
 
 
  pinMode(Enable_Pin, OUTPUT);
  digitalWrite(Enable_Pin, HIGH);  // Standby kapalı
}
 
void Forward() {
  digitalWrite(Left_motor,  LOW);
  digitalWrite(Right_motor, LOW);
  digitalWrite(Enable_Pin,  HIGH);
  analogWrite(PWM_LEFT,  150);
  analogWrite(PWM_RIGHT, 150);
}
 
void Backward() {
  digitalWrite(Left_motor,  HIGH);
  digitalWrite(Right_motor, HIGH);
  digitalWrite(Enable_Pin,  HIGH);
  analogWrite(PWM_LEFT,  150);
  analogWrite(PWM_RIGHT, 150);
}
 
void Left() {
  digitalWrite(Left_motor,  LOW);
  digitalWrite(Right_motor, HIGH);
  digitalWrite(Enable_Pin,  HIGH);
  analogWrite(PWM_LEFT,  150);
  analogWrite(PWM_RIGHT, 0);
}
 
void Right() {
  digitalWrite(Left_motor,  HIGH);
  digitalWrite(Right_motor, LOW);
  digitalWrite(Enable_Pin,  HIGH);
  analogWrite(PWM_LEFT,  0);
  analogWrite(PWM_RIGHT, 150);
}
 
void Stop() {
  digitalWrite(Left_motor,  LOW);
  digitalWrite(Right_motor, LOW);
  analogWrite(PWM_LEFT,  0);
  analogWrite(PWM_RIGHT, 0);
}

//Motor_control; regardless of which controller it comes from, it converts it to PWM+ direction in the same way. A common execution layer independent of the controller
void Motor_Control(float Speed) {
  int Final_PWM = constrain((int)abs(Speed), 0, 255);
  // Deadzonecompensation - the motordoesn't move at all below this PWM value
  if (Final_PWM > 0 && Final_PWM < deadzone) {
    Final_PWM = deadzone;  // Reach the minimum movement threshold
  }
  // Directional control
  if (Speed > 0) {
    Forward();
  } else if (Speed < 0) {
    Backward();
  } else {
    Stop();
    return;
  }
 
  analogWrite(PWM_LEFT,  Final_PWM);
  analogWrite(PWM_RIGHT, Final_PWM);
 
}