/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       humanfriend22                                             */
/*    Created:      5/3/2024, 6:28:51 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <iostream>

#include "config.h"
#include "vex.h"
#include "vpp.h"

#include "pid.h"

using namespace vpp;

vex::competition Competition;
vex::brain Brain;
Controller controller;

Motor left1(MOTOR_LEFT_1);
Motor left2(MOTOR_LEFT_2);
Motor left3(MOTOR_LEFT_3);
Motor right1(MOTOR_RIGHT_1);
Motor right2(MOTOR_RIGHT_2);
Motor right3(MOTOR_RIGHT_3);

MotorGroup leftMotorGroup({left1, left2, left3});
MotorGroup rightMotorGroup({right1, right2, right3});
vex::inertial inertialSensor(INERTIAL_SENSOR);

Drivetrain drivetrain(leftMotorGroup, rightMotorGroup);
PIDController pidController(drivetrain, inertialSensor);

void driverControl() {
    drivetrain.setStopMode(BRAKE);

    while (true) {
        int x = controller.rightX();
        int y = controller.leftY();

        drivetrain.arcade(y, x);
        wait(20);
    }
};

void autonomous() {
    inertialSensor.calibrate(20);
    while (inertialSensor.isCalibrating()) {
        wait(100);
    }
    inertialSensor.resetHeading();

    pidController.drive(600, 100, 5000);
    // pidController.manuever(DRIVE, -600, 100, 5000);

    driverControl();
};

int main() {
    // Competition.drivercontrol(driverControl);
    // Competition.autonomous(autonomous);

    autonomous();
}