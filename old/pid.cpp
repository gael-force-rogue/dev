#include "pid.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>

#include "config.h"
#include "vex.h"

float normalize180(float angle) {
    while (angle < -180) {
        angle += 360;
    }
    while (angle >= 180) {
        angle -= 360;
    }
    return angle;
};

float PIDStateManager::update(float error) {
    this->runningIntegral += error;
    this->previousError = error;

    return (error * kP) + (runningIntegral * kI) + ((error - previousError) * kD);
};

void PIDController::manuever(ManueverType manueverType, float target, float maxSpeed, float timeout) {
    // Configuration & Global Info
    auto currentStateManager = PIDStateManager(config[manueverType].kP, config[manueverType].kI, config[manueverType].kD);
    const float startingAngle = chassis.inertialSensor.heading();
    const float accuracy = config[manueverType].accuracy;

    chassis.leftCluster.stop(hold);
    chassis.rightCluster.stop(hold);

    timer safetyTimer;

    if (manueverType == DRIVE) {
        auto headingStateManager = PIDStateManager(config[HEADING].kP, config[HEADING].kI, config[HEADING].kD);
        const float leftStartingPosition = chassis.leftCluster.averagePosition();
        const float rightStartingPosition = chassis.rightCluster.averagePosition();

        do {
            float lateralError = target - ((chassis.leftCluster.averagePosition() - leftStartingPosition + chassis.rightCluster.averagePosition() - rightStartingPosition) / 2);
            float angularError = normalize180(startingAngle - chassis.inertialSensor.heading());

            float lateralMotorPower = currentStateManager.update(lateralError);
            float angularMotorPower = headingStateManager.update(angularError);

            chassis.arcade(lateralMotorPower, angularMotorPower);

            wait(20, msec);
        } while (std::abs(currentStateManager.previousError) >= accuracy && safetyTimer.time() < timeout);
    } else if (manueverType == TURN) {
        do {
            float error = normalize180(target - chassis.inertialSensor.heading());
            float motorPower = currentStateManager.update(error);

            chassis.arcade(0, motorPower);

            wait(20, msec);
        } while (abs(currentStateManager.previousError) >= accuracy || safetyTimer.time() < timeout);
    } else if (manueverType == SWING) {
        const float right = target > 0;

        do {
            float error = target - chassis.leftCluster.averagePosition();
            float motorPower = currentStateManager.update(error);

            if (right) {
                chassis.tank(0, motorPower);
            } else {
                chassis.tank(motorPower, 0);
            };

            wait(20, msec);
        } while (abs(currentStateManager.previousError) >= accuracy && safetyTimer.time() < timeout);
    };

    safetyTimer.~timer();
};