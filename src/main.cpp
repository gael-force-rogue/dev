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
#include "config.h"
#include "autons.h"
#include "devices.h"

using namespace vpp;

vex::competition Competition;
Controller mainController;
vex::brain Brain;

Motor left1(vex::PORT20, BLUE_6, false);
Motor left2(vex::PORT16, BLUE_6, true);
Motor left3(vex::PORT1, BLUE_6, false);
Motor right1(vex::PORT9, BLUE_6, true);
Motor right2(vex::PORT17, BLUE_6, false);
Motor right3(vex::PORT10, BLUE_6, true);

MotorGroup leftMotorGroup({left1, left2, left3});
MotorGroup rightMotorGroup({right1, right2, right3});

vex::pneumatics clamp(Brain.ThreeWirePort.B);
vex::pneumatics knocker(Brain.ThreeWirePort.A);

vex::rotation verticalTrackerWheel(vex::PORT5, false);
vex::rotation sidewaysTrackerWheel(vex::PORT21, true);

// 3 0.05
// 0.7

IMU imu(vex::PORT3);
TankChassis chassis = TankChassis(leftMotorGroup, rightMotorGroup, imu, 2.75, 1, 6.5)
                          .withDriveConstants(3, 0.1, 0, 5, 100, 2000, 0.5, 100)
                          .withHeadingConstants(0.7, 0, 0, 0, 20, 2000, 0.5, 0)
                          .withTurnConstants(0.5, 0, 1, 0, 75, 1000, 0.5, 200)
                          .withSwingConstants(1, 0, 3, 3, 100, 1300, 2, 100)
                          .withVerticalTrackerWheel(2, 1, -0.5, verticalTrackerWheel)
                          .withSidewaysTrackerWheel(2.75, 1, -1.25, sidewaysTrackerWheel);

Lift lift(vex::PORT18, true);
Intake intake(vex::PORT19, true, vex::optical(vex::PORT15, false));

void pistonThreadF() {
    std::cout << "Piston thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        sleep(20);
    };
};

bool odometryThreadDrivercontrolKill = false;
void odometryThreadF() {
    std::cout << "Odometry thread: " << vex::this_thread::get_id() << std::endl;

    while (!odometryThreadDrivercontrolKill) {
        chassis.odometry.update(verticalTrackerWheel.position(vex::deg), sidewaysTrackerWheel.position(vex::deg), chassis.imu.heading());
        intake.handleColorSortMacro();

        // BRAIN_PRINTF(1, "X: %f", chassis.odometry.pose.x);
        // BRAIN_PRINTF(2, "Y: %f", ch);
        // BRAIN_PRINTF(3, "Theta: %f", chassis.odometry.pose.theta);

        mainController.screen.clearScreen();
        mainController.screen.setCursor(1, 1);
        mainController.screen.print("X: %f", chassis.odometry.pose.x);
        mainController.screen.setCursor(2, 1);
        mainController.screen.print("Y: %f", chassis.odometry.pose.y);
        mainController.screen.setCursor(3, 1);
        mainController.screen.print("Theta: %f", chassis.odometry.pose.theta);

        if (!left1.connected() || !left2.connected() || !left3.connected() || !right1.connected() || !right2.connected() || !right3.connected() || !imu.connected()) {
            chassis.endMotion();
            BRAIN_PRINTLN(9, "WARNING: DEVICE DISCONNECTED");
        };

        sleep(20);
    };
};

void liftMacroThreadF() {
    std::cout << "Lift macro thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        IF_BUTTON_PRESS(mainController.ButtonUp(), lift.toggleState(), 20);
        IF_BUTTON_PRESS(mainController.ButtonA(), clamp.open(), 20);
        IF_BUTTON_PRESS(mainController.ButtonB(), clamp.close(), 20);
        // IF_BUTTON_PRESS(mainController.ButtonY(), knocker.toggle(), 20);

        sleep(20);
    };
};

void drivercontrol() {
    vex::thread pistonThread(pistonThreadF);
    vex::thread liftMacroThread(liftMacroThreadF);
    // odometryThreadDrivercontrolKill = true;
    chassis.setDefaultStopMode(COAST);

    while (true) {
        float y = mainController.leftY();
        float x = mainController.rightX();

        chassis.arcade(y, x);

        intake.handleDrivercontrol(mainController.ButtonR1(), mainController.ButtonR2());
        lift.handleDrivercontrol(mainController.ButtonL1(), mainController.ButtonL2());

        sleep(10);
    };
};

bool matchStarted = false;
Auton selectedAuton = AUTON;
void autonomous() {
    matchStarted = true;

    float start = vex::timer::system();

    switch (selectedAuton) {
        case RED_AWP:
            red_awp();
            break;
        case BLUE_AWP:
            blue_awp();
            break;
        case SKILLS:
            skills();
            break;
    };

    float end = vex::timer::system();
    std::cout << "Auton Took: " << (end - start) << std::endl;
};

int main() {
    chassis.calibrate();
    verticalTrackerWheel.resetPosition();
    sidewaysTrackerWheel.resetPosition();
    lift.setVelocity(100);
    lift.setDefaultStopMode(HOLD);

    vex::thread odometryThread(odometryThreadF);

    Competition.drivercontrol(drivercontrol);
    Competition.autonomous(autonomous);

    while (true) sleep(500);
}