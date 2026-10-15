#ifndef LQR_H
#define LQR_H
 
#include <Arduino.h>
#include "RobotState.h"

 
class LQR{
  private:
  //LQR gains- finded in MATLAB via Q=[1,1,100,1] and R=5. Uploading to Arduino.
  // Respectively [x,xdot,theta,thetadot]
  const float K_LQR_x        = -0.447214;
  const float K_LQR_xdot     = -1.036294;
  const float K_LQR_theta    = -5.170126;
  const float K_LQR_thetadot = -0.523897;
  // Torque(N.m)->PWM conversion constant. In MATLAB, the measured motor limit tau_max,actual= 0.6374 N.m,which corresponds to PWM=255 in Arduino code
  const float TORQUE_to_PWM = 255.0 /0.6374;
  //Encoder tick-to-metre conversion constant(from the motor datasheet and wheel radius)
  const float METRE_TO_TICK = 1.310e-4;

  public:
  float LQR_Compute(RobotState state);

};
 
#endif