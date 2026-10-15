#include "PID.h"

//PID control law: tau = -(K_x*x + K_xdot*xdot + K_theta*theta + K_thetadot*K_thetadot + K_z*z)
//The only difference from LQR is that z(the integral state) accumulate within itself
  
float PID::PID_Compute(RobotState state,float dt) {
  // Updating z(zdot=-x)
  z_integral += (-state.x_m)*dt;
  // Torque calculation is the same with LQR Computition,only added the fifth term
  float tau_Nm = -(K_PID_x*state.x_m + K_PID_xdot*state.xdot_ms + K_PID_theta*state.theta_rad + K_PID_thetadot*state.thetadot_rads + K_PID_z*z_integral) ;
 // N.m -> PWM, being limited
 float pwm = tau_Nm * TORQUE_to_PWM;
 pwm = constrain (pwm, -255,255);

 return pwm;

}

void PID::ResetState() {
  z_integral = 0.0;
  
}