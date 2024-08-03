#ifndef CONFIG_H
#define CONFIG_H

// Devices & Robot
#define MOTOR_LEFT_1 PORT7, BLUE_6, true
#define MOTOR_LEFT_2 PORT8, BLUE_6, false
#define MOTOR_LEFT_3 PORT3, BLUE_6, true
#define MOTOR_RIGHT_1 PORT10, BLUE_6, false
#define MOTOR_RIGHT_2 PORT12, BLUE_6, true
#define MOTOR_RIGHT_3 PORT11, BLUE_6, false

#define INERTIAL_SENSOR PORT1
#define MOTOR_INTAKE PORT6, BLUE_6, true

#define MOTOR_LIFT_1 PORT13, BLUE_6, true
#define MOTOR_LIFT_2 PORT20, BLUE_6, false

// Odometry
#define WHEEL_RADIUS 3.25

#define LEFT_ENCODER PORT1
#define RIGHT_ENCODER PORT2
#define SIDEWAYS_ENCODER PORT6

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