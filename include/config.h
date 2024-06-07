#ifndef CONFIG_H
#define CONFIG_H

// Devices & Robot
#define INERTIAL_SENSOR PORT20
#define MOTOR_LEFT_1 PORT6, true
#define MOTOR_LEFT_2 PORT10, true
#define MOTOR_LEFT_3 PORT16, true
#define MOTOR_RIGHT_1 PORT16
#define MOTOR_RIGHT_2 PORT20
#define MOTOR_RIGHT_3 PORT13

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
#define DRIVE_PID_kD 10.0
#define DRIVE_PID_SLEW 0.3
#define DRIVE_PID_ACCURACY 10.0

#define TURN_PID_kP 0.0
#define TURN_PID_kI 0.0
#define TURN_PID_kD 0.0
#define TURN_PID_SLEW 0.0
#define TURN_PID_ACCURACY 0.0

#define SWING_PID_kP 0.0
#define SWING_PID_kI 0.0
#define SWING_PID_kD 0.0
#define SWING_PID_SLEW 0.0
#define SWING_PID_ACCURACY 0.0

#endif // CONFIG_H