 #ifndef PID_H
#define PID_H
 
#include <Arduino.h>
#include "RobotState.h"
 
class PID {
 
private:
 //Model-based parallel PID gains: Determined using pole placement in MATLAB. [x,xdot,theta,thetadot,z] ; zdot =-x
 const float K_PID_x        = -0.304472;
 const float K_PID_xdot     = -0.138352;
 const float K_PID_theta    = -0.317058;
 const float K_PID_thetadot = -0.026825;
 const float K_PID_z        = 0.315673;

 const float TORQUE_to_PWM = 255.0 / 0.6374;
 float z_integral = 0.0; // Unlike LQR, PID accumulates a state,"memory" within itself. That's why dt is needed on every call(to calculate the accumulation correctly)

public:
 float PID_Compute(RobotState state, float dt);
 void ResetState(); // When the robot falls, the z integral becomes zero
 float GetZ(){
  return z_integral; // For debug
 }
 
};
 
#endif