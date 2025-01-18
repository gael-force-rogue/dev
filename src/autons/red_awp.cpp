#include "config.h"

void unclampThreadF() {
    sleep(1200);
    clamp.close();
    intake.forward();
};

void delayedLiftThreadF() {
    sleep(1000);
    lift.spinToPosition(LOAD, true);
};

void red_awp() {
    setBackwardsPIDConstants();
    chassis.driveDistance(-28, 1);
    setPIDConstants();
    clamp.open();
    sleep(700);
    START_CHAIN
    chassis.turnToAngle(120);
    intake.forward();
    chassis.driveDistance(17);

    chassis.rightSwingToAngle(23);
    chassis.driveDistance(15);

    chassis.turnToAngle(-80);
    intake.stop();
    vex::thread unclampThread(unclampThreadF);
    chassis.driveDistance(27, 1);
    chassis.turnToAngle(-123);
    chassis.driveAlgorithm.constants.maxSpeed = 40;
    vex::thread delayedLiftThread(delayedLiftThreadF);
    chassis.driveDistance(30);
    sleep(750);
    setBackwardsPIDConstants();
    chassis.driveDistance(-15, 1);
    setPIDConstants();
    chassis.turnToAngle(-39);
    chassis.driveDistance(9);
    END_CHAIN
};