#ifndef CONFIG_H
#define CONFIG_H

#include "vex.h"

using namespace vex;

// Devices
#define MOTOR_LEFT_1 PORT2, true
#define MOTOR_LEFT_2 PORT8, false
#define MOTOR_LEFT_3 PORT6, true
#define MOTOR_RIGHT_1 PORT9, false
#define MOTOR_RIGHT_2 PORT15, true
#define MOTOR_RIGHT_3 PORT11, false

#define MOTOR_INTAKE PORT16, true
#define MOTOR_LIFT PORT7, false
#define DISTANCE_SENSOR PORT5

#define INERTIAL_SENSOR PORT12

// Odometry
#define WHEEL_RADIUS 3.25

#define LEFT_TRACKER_OFFSET 5.125
#define RIGHT_TRACKER_OFFSET 5
#define SIDEWAYS_TRACKER_OFFSET 6.5

// PID Constants
#define DRIVE_PID_kP 0.6
#define DRIVE_PID_kI 0.0
#define DRIVE_PID_kD 8.0
#define DRIVE_PID_ACCURACY 1

// no clmap kP: 1.0

#define HEADING_PID_kP 30
#define HEADING_PID_kI 0.0
#define HEADING_PID_kD 0.0
#define HEADING_PID_ACCURACY 0.5

#define TURN_PID_kP 1.0
#define TURN_PID_kI 0.0
#define TURN_PID_kD 0.0
#define TURN_PID_ACCURACY 0.5

#define SWING_PID_kP 0.0
#define SWING_PID_kI 0.0
#define SWING_PID_kD 0.0
#define SWING_PID_ACCURACY 0.0

#endif  // CONFIG_H