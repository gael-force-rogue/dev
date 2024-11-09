#include "vpp/chassis.h"
#include "vpp/algorithms/pid.h"

#include <iostream>

using namespace vpp;

void TankChassis::resetToInitialConstants() {
    this->driveConstants = this->defaultDriveConstants;
    this->headingConstants = this->defaultHeadingConstants;
    this->turnConstants = this->defaultTurnConstants;
    this->swingConstants = this->defaultSwingConstants;
};

void TankChassis::driveDistance(float distance, bool followThrough) {
    PIDAlgorithm driveAlgorithm(this->driveConstants);
    PIDAlgorithm headingAlgorithm(this->headingConstants);

    const float startingHeading = imu.heading();
    const float startingPosition = (leftGroup.averagePosition() + rightGroup.averagePosition()) / 2;

    while (!driveAlgorithm.isSettled()) {
        float lateralError = distance + startingPosition - ((leftGroup.averagePosition() + rightGroup.averagePosition()) / 2);
        float angularError = startingHeading - imu.heading();

        float lateralPower = driveAlgorithm.update(lateralError);
        float angularPower = headingAlgorithm.update(angularError);

        arcade(lateralPower, angularPower);

        sleep(10);
    };

    if (!followThrough) {
        stop(HOLD);
    }
};

void TankChassis::driveDistance(float distance, float maxSpeed, bool followThrough) {
    this->driveConstants.maxSpeed = maxSpeed;
    driveDistance(distance, followThrough);
    resetToInitialConstants();
};

void TankChassis::turnToAngle(float angle, bool followThrough) {
    PIDAlgorithm turnAlgorithm(this->turnConstants);

    angle = normalize180(angle);

    while (!turnAlgorithm.isSettled()) {
        float error = angle - imu.heading();
        float power = turnAlgorithm.update(error);

        arcade(0, power);

        sleep(10);
    };

    if (!followThrough) {
        stop(HOLD);
    }
};

void TankChassis::turnToAngle(float angle, float maxSpeed, bool followThrough) {
    this->turnConstants.maxSpeed = maxSpeed;
    turnToAngle(angle, followThrough);
    resetToInitialConstants();
};

void TankChassis::leftSwingToAngle(float angle, bool followThrough) {
    PIDAlgorithm swingAlgorithm(this->swingConstants);

    angle = normalize180(angle);

    while (!swingAlgorithm.isSettled()) {
        float error = angle - imu.heading();
        float power = swingAlgorithm.update(error);

        tank(0, power);

        sleep(10);
    };

    if (!followThrough) {
        stop(HOLD);
    }
};

void TankChassis::leftSwingToAngle(float angle, float maxSpeed, bool followThrough) {
    this->swingConstants.maxSpeed = maxSpeed;
    leftSwingToAngle(angle, followThrough);
    resetToInitialConstants();
};

void TankChassis::rightSwingToAngle(float angle, bool followThrough) {
    PIDAlgorithm swingAlgorithm(this->swingConstants);

    angle = normalize180(angle);

    while (!swingAlgorithm.isSettled()) {
        float error = angle - imu.heading();
        float power = swingAlgorithm.update(error);

        tank(power, 0);

        sleep(10);
    };

    if (!followThrough) {
        stop(HOLD);
    }
};

void TankChassis::rightSwingToAngle(float angle, float maxSpeed, bool followThrough) {
    this->swingConstants.maxSpeed = maxSpeed;
    rightSwingToAngle(angle, followThrough);
    resetToInitialConstants();
};