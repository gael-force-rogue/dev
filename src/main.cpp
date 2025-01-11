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

vex::competition Competition;
Controller MainController;
vex::brain Brain;

using namespace vpp;

Motor left1(vex::PORT20, BLUE_6, false);
Motor left2(vex::PORT16, BLUE_6, true);
Motor left3(vex::PORT1, BLUE_6, false);
Motor right1(vex::PORT9, BLUE_6, true);
Motor right2(vex::PORT17, BLUE_6, false);
Motor right3(vex::PORT10, BLUE_6, true);

MotorGroup leftMotorGroup({left1, left2, left3});
MotorGroup rightMotorGroup({right1, right2, right3});

Pneumatic clamp(Brain.ThreeWirePort.G);
Pneumatic knocker(Brain.ThreeWirePort.H);

IMU imu(vex::PORT3);
TankChassis chassis = TankChassis(leftMotorGroup, rightMotorGroup, imu, 4, 1, 10.2)
                          .withDriveConstants(22, 0, 40, 1, 100, 5000, 1, 100)
                          .withHeadingConstants(0.8, 0, 0, 0, 20, 2000, 0.5, 0)
                          .withTurnConstants(1.1, 0, 1.8, 0, 75, 1300, 2, 200)
                          .withSwingConstants(1.5, 0, 0, 3, 100, 1300, 2, 100)
                          .withVerticalTrackerWheel(2, 1, 2)
                          .withSidewaysTrackerWheel(2, 1, 2);

Lift lift(vex::PORT18, true);
Intake intake(vex::PORT19, true, vex::optical(vex::PORT15, false), lift);

vex::rotation verticalTrackerWheel(vex::PORT5, false);
vex::rotation sidewaysTrackerWheel(vex::PORT21, false);

void pistonThreadF() {
    std::cout << "Piston thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        IF_BUTTON_PRESS(MainController.ButtonA(), clamp.open(), 20);
        IF_BUTTON_PRESS(MainController.ButtonB(), clamp.close(), 20);
        IF_BUTTON_PRESS(MainController.ButtonY(), knocker.toggle(), 20);

        sleep(20);
    };
};

bool odometryThreadDrivercontrolKill = false;
void odometryThreadF() {
    std::cout << "Odometry thread: " << vex::this_thread::get_id() << std::endl;

    while (!odometryThreadDrivercontrolKill) {
        chassis.odometry.update(verticalTrackerWheel.position(vex::deg), sidewaysTrackerWheel.position(vex::deg), chassis.imu.heading());
        intake.handleColorSortMacro();

        BRAIN_PRINTF(1, "X: %f", chassis.odometry.pose.x);
        BRAIN_PRINTF(2, "Y: %f", chassis.odometry.pose.y);
        BRAIN_PRINTF(3, "Theta: %f", chassis.odometry.pose.theta);

        if (!left1.connected() || !left2.connected() || !right1.connected() || !right2.connected() || !imu.connected()) {
            chassis.endMotion();
            BRAIN_PRINTLN(9, "WARNING: DEVICE DISCONNECTED");
        };

        sleep(20);
    };
};

void liftMacroThreadF() {
    std::cout << "Lift macro thread: " << vex::this_thread::get_id() << std::endl;

    while (true) {
        if (MainController.ButtonUp()) {
            lift.toggleState();

            while (MainController.ButtonUp()) sleep(20);
        };

        sleep(20);
    };
};

void drivercontrol() {
    vex::thread pistonThread(pistonThreadF);
    vex::thread liftMacroThread(liftMacroThreadF);
    odometryThreadDrivercontrolKill = true;
    chassis.setDefaultStopMode(COAST);

    while (true) {
        float y = MainController.leftY();
        float x = MainController.rightX();

        chassis.arcade(y, x);

        intake.handleDrivercontrol(MainController.ButtonR1(), MainController.ButtonR2());
        lift.handleDrivercontrol(MainController.ButtonL1(), MainController.ButtonL2());

        sleep(10);
    };
};

void autonomous() {
    switch (AUTON) {
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
};

int main() {
    MAIN_THREAD_ID = vex::this_thread::get_id();
    chassis.calibrate();
    vex::thread odometryThread(odometryThreadF);

    if (Competition.isCompetitionSwitch()) {
        Competition.drivercontrol(drivercontrol);
        Competition.autonomous(autonomous);
    } else {
        drivercontrol();
    };

    // while (true) sleep(500);
}