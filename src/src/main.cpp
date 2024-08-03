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

// Controllers & Algorithms
PIDController pidController(drivetrain, inertialSensor);

void clampTask() {
    controller.vibrate(". .");
    bool clampIsActive = false;
    clamp.close();

    std::cout << "Clamp task: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        if (controller.B()) {
            if (clampIsActive) {
                clamp.open();
            } else {
                clamp.close();
            }

            clampIsActive = !clampIsActive;
            do {
                vex::this_thread::sleep_for(20);
            } while (controller.B());
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

        if (controller.R1()) {
            intake.spin(100);
        } else if (controller.R2()) {
            intake.spin(-100);
        } else if (controller.L1()) {
            lift.spin(100);
        } else if (controller.L2()) {
            lift.spin(-100);
        } else {
            intake.stop();
            lift.stop();
        };

        vex::this_thread::sleep_for(20);
    };
};

// void odometryTask() {
//     std::cout << "Odometry task: " << vex::this_thread::get_id() << std::endl;

//     while (true) {
//         drivetrain.update();
//         wait(20);
//     }
// };

void autonomous() {
    inertialSensor.calibrate();

    drivercontrol();
};

int main() {
    // autonomous();
    drivercontrol();
}