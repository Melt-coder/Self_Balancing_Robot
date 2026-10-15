#include "LQR.h"

//LQR control law : Tau = -(K_x*x + K_xdot*xdot + K_theta*theta + K_thetadot*K_thetadot)
//Input: robot's 4 states, SI units (metre, metre/sec, radyan, radyan/sec)
//Output: PWM value to be sent to the motor(between -255 and 255)

float LQR:: LQR_Compute(RobotState state) {

  // Torque calculating (N.m),same formula to MATLAB "u=-K*x" 
  float tau_Nm = -(K_LQR_x*state.x_m + K_LQR_xdot*state.xdot_ms + K_LQR_theta*state.theta_rad + K_LQR_thetadot*state.thetadot_rads) ;

  // Converting from torque to PWM
  float pwm = tau_Nm * TORQUE_to_PWM;

  // To trim the output if it exceeds the motor's limit
  pwm = constrain (pwm, -255,255);

  return pwm;

}