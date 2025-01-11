// Define's all devices and classes that autons or devices may need
// This is the only file that every project file should need

#pragma once

#include "vpp.h"
#include "devices.h"

int MAIN_THREAD_ID;

extern Motor left1, left2;
extern Motor right1, right2;
extern MotorGroup leftMotorGroup, rightMotorGroup;

extern Pneumatic clamp, hang, knocker;

extern IMU imu;
extern TankChassis chassis;

extern Intake intake;