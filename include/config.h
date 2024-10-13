#ifndef CONFIG_H
#define CONFIG_H

// Devices
// #define MOTOR_LEFT_1 -2, BLUE_6
// #define MOTOR_LEFT_2 6, BLUE_6
// #define MOTOR_LEFT_3 8, BLUE_6
// #define MOTOR_RIGHT_1 9, BLUE_6
// #define MOTOR_RIGHT_2 11, BLUE_6
// #define MOTOR_RIGHT_3 -15, BLUE_6

#define MOTOR_LEFT_1 -2
#define MOTOR_LEFT_2 8
#define MOTOR_LEFT_3 -6
#define MOTOR_RIGHT_1 9
#define MOTOR_RIGHT_2 -15
#define MOTOR_RIGHT_3 11

#define MOTOR_INTAKE -16
#define MOTOR_LIFT 1
#define DISTANCE_SENSOR 5

#define INERTIAL_SENSOR 12

// Odometry
#define WHEEL_RADIUS 3.25

#define LEFT_TRACKER_OFFSET 5.125
#define RIGHT_TRACKER_OFFSET 5
#define SIDEWAYS_TRACKER_OFFSET 6.5

// PID Constants
#define DRIVE_PID_kP 0.1
#define DRIVE_PID_kI 0.0
#define DRIVE_PID_kD 0.0
#define DRIVE_PID_ACCURACY 1.0

#define HEADING_PID_kP 0.2
#define HEADING_PID_kI 0.0
#define HEADING_PID_kD 0.0
#define HEADING_PID_ACCURACY 0.5

#define TURN_PID_kP 0.37
#define TURN_PID_kI 0.0
#define TURN_PID_kD 0.0
#define TURN_PID_ACCURACY 0.1

#define SWING_PID_kP 0.0
#define SWING_PID_kI 0.0
#define SWING_PID_kD 0.0
#define SWING_PID_ACCURACY 0.0

#endif  // CONFIG_H