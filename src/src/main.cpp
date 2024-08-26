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
#include "odom.h"

using namespace vpp;

// Brain & Controller
vex::competition Competition;
vex::brain Brain;
Controller controller;

// Drivetrain
Motor left1(MOTOR_LEFT_1);
Motor left2(MOTOR_LEFT_2);
Motor left3(MOTOR_LEFT_3);
Motor right1(MOTOR_RIGHT_1);
Motor right2(MOTOR_RIGHT_2);
Motor right3(MOTOR_RIGHT_3);

MotorGroup leftMotorGroup({left1, left2, left3});
MotorGroup rightMotorGroup({right1, right2, right3});
Drivetrain drivetrain(leftMotorGroup, rightMotorGroup);

// Devices
Motor lift1(MOTOR_LIFT_1);
Motor lift2(MOTOR_LIFT_2);

MotorGroup lift({lift1, lift2});
Motor intake(MOTOR_INTAKE);
vex::pneumatics clamp(Brain.ThreeWirePort.H);
vex::inertial inertialSensor(INERTIAL_SENSOR);

// Autonomous
PIDController pidController(drivetrain, inertialSensor);
Odometry odometry(LEFT_TRACKER_OFFSET, RIGHT_TRACKER_OFFSET, WHEEL_RADIUS);

void odometryTask() {
    std::cout << "Odometry thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        odometry.update(leftMotorGroup.averagePosition(), rightMotorGroup.averagePosition(), inertialSensor.heading());
        sleep(10);
    }
};

void autonomous() {
    inertialSensor.calibrate();

    vex::thread odometryThread(odometryTask);

    pidController.drive(1000, 100, 5000);
};

// Driver Control
void clampTask() {
    controller.vibrate(". .");
    bool clampIsActive = false;
    clamp.close();

    std::cout << "Clamp thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        if (controller.ButtonB()) {
            if (clampIsActive) {
                clamp.open();
            } else {
                clamp.close();
            }

            clampIsActive = !clampIsActive;
            do {
                vex::this_thread::sleep_for(20);
            } while (controller.ButtonB());
        };
    }
}

void drivercontrol() {
    std::cout << "Main thread: " << vex::this_thread::get_id() << std::endl;

    vex::thread clampThread(clampTask);

    drivetrain.setStopMode(COAST);
    while (true) {
        float y = controller.leftY();
        float x = controller.rightX();

        drivetrain.arcade(y, x * 0.6);

        if (controller.ButtonR1()) {
            intake.spin(100);
        } else if (controller.ButtonR2()) {
            intake.spin(-100);
        } else if (controller.ButtonL1()) {
            lift.spin(100);
        } else if (controller.ButtonL2()) {
            lift.spin(-100);
        } else {
            intake.stop();
            lift.stop();
        };

        sleep(20);
    };
};

// Main
int main() {
    if (Competition.isCompetitionSwitch()) {
        Competition.drivercontrol(drivercontrol);
        Competition.autonomous(autonomous);
    } else {
        // autonomous();
        drivercontrol();
    }

    while (true) sleep(5000);
}