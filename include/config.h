// Define's all devices and classes that autons or devices may need
// This is the only file that every project file should need

#pragma once

#include "vpp.h"
#include "autons.h"
#include "devices.h"

extern Motor left1, left2, left3;
extern Motor right1, right2, right3;
extern MotorGroup leftMotorGroup, rightMotorGroup;

extern vex::pneumatics clamp, knocker;

extern IMU imu;
extern TankChassis chassis;

extern Intake intake;

void setPIDConstants();
void setBackwardsPIDConstants();