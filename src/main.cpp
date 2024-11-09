/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       humanfriend22                                             */
/*    Created:      5/3/2024, 6:28:51 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <iostream>

#include "vpp.h"
#include "vpp/helpers.h"
#include "devices.h"
#include "autons.h"
#include "config.h"

using namespace vpp;

vex::competition Competition;
vex::brain Brain;
Controller mainController;

Motor left1(MOTOR_LEFT_1);
Motor left2(MOTOR_LEFT_2);
Motor left3(MOTOR_LEFT_3);
Motor right1(MOTOR_RIGHT_1);
Motor right2(MOTOR_RIGHT_2);
Motor right3(MOTOR_RIGHT_3);

MotorGroup leftMotorGroup({left1, left2, left3});
MotorGroup rightMotorGroup({right1, right2, right3});

vex::optical colorSensor(PORT2);
Intake intake = Intake(MOTOR_INTAKE).withColorSorting(colorSensor, BLUE);
Lift lift(MOTOR_LIFT, -15, intake);
IMU imu(INERTIAL_SENSOR);

Pneumatic clamp(Brain.ThreeWirePort.H);
Pneumatic hang(Brain.ThreeWirePort.B);
Pneumatic knocker(Brain.ThreeWirePort.A);

Odometry odometry = Odometry(6.25, 3);
TankChassis &chassis = TankChassis(leftMotorGroup, rightMotorGroup, imu)
                           .withDriveConstants(2, 0, 3, 100, 1000, 1, 200, 3)
                           .withHeadingConstants(2, 0, 3, 100, 1000, 1, 200, 3)
                           .withTurnConstants(2, 0, 3, 100, 1000, 1, 200, 3)
                           .withOdometry(odometry);

void intakeThreadF() {
    intake.searchForEnemyRings();
};

void pistonThreadF() {
    std::cout << "Piston thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        IF_BUTTON_PRESS(mainController.ButtonA(), clamp.open(), 20);
        IF_BUTTON_PRESS(mainController.ButtonB(), clamp.close(), 20);
        IF_BUTTON_PRESS(mainController.ButtonY(), knocker.toggle(), 20);
        IF_BUTTON_PRESS(mainController.ButtonUp(), hang.toggle(), 20);

        sleep(20);
    }
};

void drivercontrol() {
    vex::thread pistonThread(pistonThreadF);
    vex::thread intakeThread(intakeThreadF);

    chassis.setDefaultStopMode(COAST);
    intake.setDefaultStopMode(HOLD);
    lift.setDefaultStopMode(HOLD);

    while (true) {
        float y = mainController.leftY();
        float x = mainController.rightX();

        chassis.arcade(y, x * 0.3);

        intake.handleDrivercontrol(mainController.ButtonR1(), mainController.ButtonR2());
        lift.handleDrivercontrol(mainController.ButtonL1(), mainController.ButtonL2());

        sleep(20);
    };
};

void autonomous() {
    imu.calibrate();
    chassis.odometry.resetPose();
    vex::thread intakeThread(intakeThreadF);

    red_solo_awp();
};

int main() {
    if (Competition.isCompetitionSwitch()) {
        Competition.drivercontrol(drivercontrol);
        Competition.autonomous(autonomous);
    } else {
        drivercontrol();
    }

    while (true) sleep(500);
}