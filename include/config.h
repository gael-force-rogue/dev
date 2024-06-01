#ifndef CONFIG_H
#define CONFIG_H

#define INERTIAL_SENSOR PORT20

#define MOTOR_LEFT_1 PORT12
#define MOTOR_LEFT_2 PORT4
#define MOTOR_LEFT_3 PORT16, true
#define MOTOR_RIGHT_1 PORT15, true
#define MOTOR_RIGHT_2 PORT14, true
#define MOTOR_RIGHT_3 PORT13

#define DRIVE_PID_kD 0.0
#define DRIVE_PID_kI 0.0
#define DRIVE_PID_kP 0.0
#define DRIVE_PID_SLEW 0.0
#define DRIVE_PID_ACCURACY 0.0
#define DRIVE_PID_MAX_SPEED 0.0

#define TURN_PID_kD 0.0
#define TURN_PID_kI 0.0
#define TURN_PID_kP 0.0
#define TURN_PID_SLEW 0.0
#define TURN_PID_ACCURACY 0.0
#define TURN_PID_MAX_SPEED 0.0

#define SWING_PID_kD 0.0
#define SWING_PID_kI 0.0
#define SWING_PID_kP 0.0
#define SWING_PID_SLEW 0.0
#define SWING_PID_ACCURACY 0.0
#define SWING_PID_MAX_SPEED 0.0

#endif // CONFIG_H