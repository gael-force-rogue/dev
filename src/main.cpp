/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       humanfriend22                                             */
/*    Created:      5/3/2024, 6:28:51 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "config.h"
#include "motor.h"
#include "odom.h"
#include "pid.h"
#include "tuner.h"
#include "vex.h"
#include <functional>
#include <iostream>

using namespace vex;

competition Competition;
brain Brain;
controller Controller;

motor left1(MOTOR_LEFT_1);
motor left2(MOTOR_LEFT_2);
// motor left3(MOTOR_LEFT_3);
motor right1(MOTOR_RIGHT_1);
motor right2(MOTOR_RIGHT_2);
// motor right3(MOTOR_RIGHT_3);

MotorCluster leftMotorCluster(left1, left2);
MotorCluster rightMotorCluster(right1, right2);
inertial inertialSensor(INERTIAL_SENSOR);

Chassis chassis(leftMotorCluster, rightMotorCluster, inertialSensor);

PIDController pidController(chassis);
OdometryController odomController(LEFT_TRACKER_OFFSET, RIGHT_TRACKER_OFFSET, SIDEWAYS_TRACKER_OFFSET, WHEEL_RADIUS);

void autonomous() {
};

void driverControl() {
    leftMotorCluster.setDefaultBrakeMode(coast);
    rightMotorCluster.setDefaultBrakeMode(coast);

    while (true) {
        int x = Controller.Axis3.position();
        int y = Controller.Axis1.position();

        leftMotorCluster.spin(x + y);
        rightMotorCluster.spin(x - y);
    }
};

int main() {
    Competition.drivercontrol(driverControl);
    Competition.autonomous(autonomous);

    ConfigTuner configTuner(Controller, 0.1, {"kP", "kI", "kD", "slew", "accuracy", "target", "maxSpeed", "timeout"}, {0.1, 0, 0, 0.1, 5, 1000, 100, 5000});

    while (true) {
        auto action = configTuner.checkForAction();
        if (action == START) {
            auto config = configTuner.config();
            pidController.manuever({DRIVE, config["kP"], config["kI"], config["kD"], config["slew"], config["accuracy"]}, config["target"], config["maxSpeed"], config["timeout"]);
        } else if (action == MODIFIED) {
        }
        wait(100, msec);
    }
}