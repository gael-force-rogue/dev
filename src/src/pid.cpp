#include "vpp.h"
#include "pid.h"
#include <iostream>

using namespace vpp;

// Returns the reference angle for 180 degrees
float normalize180(float angle) {
    while (angle < -180) {
        angle += 360;
    }
    while (angle >= 180) {
        angle -= 360;
    }
    return angle;
};

void PIDController::drive(float target, float maxSpeed, float timeout) {
    PIDAlgorithm algorithm(config.drive, timeout);
    PIDAlgorithm headingAlgorithm(config.heading);

    const float leftStartingPosition = drivetrain.leftGroup.averagePosition();
    const float rightStartingPosition = drivetrain.rightGroup.averagePosition();
    const float startingAngle = inertialSensor.heading();

    while (algorithm.shouldContinue()) {
        float lateralError = target - ((drivetrain.leftGroup.averagePosition() - leftStartingPosition + drivetrain.rightGroup.averagePosition() - rightStartingPosition) / 2);
        float angularError = normalize180(startingAngle - inertialSensor.heading());

        float lateralMotorPower = algorithm.update(lateralError);
        float angularMotorPower = headingAlgorithm.update(angularError);

        drivetrain.arcade(lateralMotorPower, angularMotorPower);

        wait(20);
    };
};