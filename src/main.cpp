/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       humanfriend22                                             */
/*    Created:      5/3/2024, 6:28:51 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <iostream>

#include "vex.h"
#include "vpp.h"
#include "pid.h"
#include "vpp/tuner.h"

#include "config.h"

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
Motor intake(MOTOR_INTAKE);
vex::inertial inertialSensor(INERTIAL_SENSOR);

vex::pneumatics clamp(Brain.ThreeWirePort.H);
vex::pneumatics hang(Brain.ThreeWirePort.B);
vex::pneumatics knocker(Brain.ThreeWirePort.A);

// Odometry odometry(LEFT_TRACKER_OFFSET, RIGHT_TRACKER_OFFSET, WHEEL_RADIUS);
Chassis chassis(leftMotorGroup, rightMotorGroup);
PIDController pidController(chassis, inertialSensor);

// Threads
void motorThreadF() {
    std::cout << "Motor thread: " << vex::this_thread::get_id() << std::endl;

    intake.setDefaultStopMode(vpp::MotorStopMode::COAST);

    while (true) {
        if (controller.ButtonR1()) {
            intake.spin(100);
        } else if (controller.ButtonR2()) {
            intake.spin(-100);
        } else {
            intake.stop();
        }

        if (controller.ButtonL1()) {
            lift.spin(100);
        } else if (controller.ButtonL2()) {
            lift.spin(-100);
        } else {
            lift.stop();
        }

        sleep(20);
    }
};

void pistonThreadF() {
    std::cout << "Piston thread: " << vex::this_thread::get_id() << std::endl;

    bool hangIsOpen = true;
    bool knockerIsOpen = false;

    while (true) {
        BUTTON_PRESS(controller.ButtonA(), clamp.open(), 20);
        BUTTON_PRESS(controller.ButtonB(), clamp.close(), 20);
        if (controller.ButtonY()) {
            if (knockerIsOpen) {
                knocker.close();
            } else {
                knocker.open();
            }
            knockerIsOpen = !knockerIsOpen;
            while (controller.ButtonY()) {
                sleep(20);
            }
        }
        if (controller.ButtonUp()) {
            if (hangIsOpen) {
                hang.close();
            } else {
                hang.open();
            }
            hangIsOpen = !hangIsOpen;
            while (controller.ButtonUp()) {
                sleep(20);
            }
        }

        sleep(20);
    }
};

void imuThreadF() {
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Heading: %f", inertialSensor.heading());

        vpp::sleep(50);
    }
}

// Driver & Autonomous
void autonomous() {
    chassis.leftGroup.setDefaultStopMode(BRAKE);
    chassis.rightGroup.setDefaultStopMode(BRAKE);

    // 600 = 2.5 tiles
    // 600/5 = 0.5 tiles
    // 120 = 0.5 tiles
    // 240 = 1 tile

    // 1 Tile
    // pidController.drive(370, 50, 5000, 2);
    pidController.turn(5, 70, 10000, 50);
};

void drivercontrol() {
    std::cout << "Driver Control: " << vex::this_thread::get_id() << std::endl;

    vex::thread motorThread(motorThreadF);
    vex::thread pistonThread(pistonThreadF);

    chassis.setDefaultStopMode(COAST);
    while (true) {
        float y = controller.leftY();
        float x = controller.rightX();

        chassis.arcade(y, x * 0.3);

        sleep(20);
    };
};

int main() {
    std::cout << "Main thread: " << vex::this_thread::get_id() << std::endl;

    // vex::thread tunerThread(tunerThreadF);
    vex::thread imuThread(imuThreadF);

    if (Competition.isCompetitionSwitch()) {
        Competition.drivercontrol(drivercontrol);
        Competition.autonomous(autonomous);
    } else {
        drivercontrol();
    }

    while (true) sleep(500);
}