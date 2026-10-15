#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H
 

// The single interface produced by the measurement layer and consumed by the control layer
// Both LQR and PID perceive the robot^s state solely thorough these four numbers

struct RobotState{ // I defined a 'struct' because I want to create a shared data container; this allows both LQR and PID controllers to access the measured data where needed.
float x_m; // posion (metre)
float xdot_ms; // velocity (metre/sec)
float theta_rad;// angle (radyan)
float thetadot_rads;// angular velocity (radyan/sec)
};
 
#endif