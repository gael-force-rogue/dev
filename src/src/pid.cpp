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
    PIDAlgorithm headingAlgorithm(config.heading, timeout);

    std::cout << algorithm.timeout << std::endl;

    const float leftStartingPosition = drivetrain.leftGroup.averagePosition();
    const float rightStartingPosition = drivetrain.rightGroup.averagePosition();
    const float startingAngle = inertialSensor.heading();

    do {
        float lateralError = target - ((drivetrain.leftGroup.averagePosition() - leftStartingPosition + drivetrain.rightGroup.averagePosition() - rightStartingPosition) / 2);
        float angularError = normalize180(startingAngle - inertialSensor.heading());

        float lateralMotorPower = algorithm.update(lateralError);
        float angularMotorPower = headingAlgorithm.update(angularError);

        std::cout << "Angular Error: " << angularError << std::endl;
        std::cout << "Angular Motor Power: " << angularMotorPower << std::endl;

        drivetrain.arcade(lateralMotorPower, angularMotorPower);

        wait(20);
    } while (algorithm.shouldContinue());

    std::cout << "Done" << std::endl;
};

void PIDController::turn(float target, float maxSpeed, float timeout) {
    PIDAlgorithm algorithm(config.turn, timeout);

    const float startingAngle = inertialSensor.heading();

    do {
        float error = normalize180(target - (inertialSensor.heading() - startingAngle));
        float motorPower = algorithm.update(error);

        drivetrain.arcade(0, motorPower, maxSpeed);

        wait(20);
    } while (algorithm.shouldContinue());
};