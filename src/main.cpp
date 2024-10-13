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

#include "intake.h"

using namespace vpp;

// Devices
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

Motor lift(MOTOR_LIFT);
vex::pneumatics clamp(Brain.ThreeWirePort.H);
vex::pneumatics hang(Brain.ThreeWirePort.B);
vex::inertial inertialSensor(INERTIAL_SENSOR);
Motor intake(MOTOR_INTAKE);

Odometry odometry(LEFT_TRACKER_OFFSET, RIGHT_TRACKER_OFFSET, WHEEL_RADIUS);
Chassis chassis(leftMotorGroup, rightMotorGroup, odometry);

// Threads
void odometryThreadF() {
    std::cout << "Odometry thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        odometry.update(leftMotorGroup.averagePosition(), rightMotorGroup.averagePosition(), inertialSensor.heading());

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("X: %f", odometry.pose.x);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Y: %f", odometry.pose.y);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Theta: %f", odometry.pose.theta);

        sleep(10);
    }
};

void intakeThreadF() {
    std::cout << "Intake thread: " << vex::this_thread::get_id() << std::endl;

    intake.setDefaultStopMode(vpp::MotorStopMode::COAST);

    while (true) {
        if (controller.ButtonR1()) {
            intake.spin(100);

            do {
                vex::this_thread::sleep_for(20);
            } while (controller.ButtonR1());

        } else if (controller.ButtonR2()) {
            intake.spin(-100);

            do {
                vex::this_thread::sleep_for(20);
            } while (controller.ButtonR2());

        } else {
            intake.stop();
        }
        sleep(20);
    }
};

void liftThreadF() {
    std::cout << "Lift thread: " << vex::this_thread::get_id() << std::endl;

    lift.setDefaultStopMode(vpp::MotorStopMode::COAST);

    while (true) {
        if (controller.ButtonL1()) {
            lift.spin(100);
        } else if (controller.ButtonL2()) {
            lift.spin(-100);
        } else {
            lift.stop();
        }
        sleep(20);
    };
};

void clampThreadF() {
    std::cout << "Clamp thread: " << vex::this_thread::get_id() << std::endl;
    clamp.open();

    bool clampIsActive = false;

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
};

// Driver & Autonomous
void autonomous() {
    inertialSensor.calibrate();

    vex::thread odometryThread(odometryThreadF);

    // chassis.moveToPoint(Pose(2, 3, 5), 3, 5000);
};

void drivercontrol() {
    std::cout << "Main thread: " << vex::this_thread::get_id() << '\n';

    // vex::thread intakeThread(intakeThreadF);
    // vex::thread clampThread(clampThreadF);
    // vex::thread liftThread(liftThreadF);
    // vex::thread odometryThread(odometryThreadF);

    chassis.setDefaultStopMode(COAST);
    while (true) {
        float y = controller.leftY();
        float x = controller.rightX();

        chassis.arcade(y, x);

        sleep(20);
    };
};

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