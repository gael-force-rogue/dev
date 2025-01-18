#include "vpp/chassis.h"
#include "vpp/algorithms/pid.h"
#include "vpp/init.h"

#include <iostream>

using namespace vpp;

void TankChassis::resetConstants() {
    this->driveAlgorithm.resetConstants();
    this->headingAlgorithm.resetConstants();
    this->turnAlgorithm.resetConstants();
    this->swingAlgorithm.resetConstants();
    this->arcAlgorithm.resetConstants();
};

void TankChassis::calibrate() {
    this->imu.calibrate();
    this->imu.reset();
    this->leftGroup.resetPosition();
    this->rightGroup.resetPosition();
    this->odometry.resetPose();
};

void TankChassis::driveDistance(float distance, float earlyExitError) {
    // INCHES TO DEGREES
    // distance = (fraction of circle) * (circumference)
    // distance = (degrees / 360) * (pi * diameter)
    // distance = (4 * pi / 360) * degrees
    // degrees = distance / (4 * pi / 360)

    // distance = distance / (wheelDiameter * M_PI / (360 * externalRatio));
    // distance *= inchesToDegreesConversionFactor;

    motionIsActive = true;
    driveAlgorithm.reset();
    headingAlgorithm.reset();

    const float startingHeading = imu.heading();
    const float startingPosition = odometry.verticalPositionInInches();

    while (!driveAlgorithm.isSettled() && motionIsActive) {
        float lateralError = distance + startingPosition - odometry.verticalPositionInInches();
        float angularError = startingHeading - imu.heading();

        if (earlyExitError != 0 && fabs(lateralError) < earlyExitError) break;

        float lateralPower = driveAlgorithm.update(lateralError);
        float angularPower = headingAlgorithm.update(angularError);

        std::cout << lateralError << std::endl;

        arcade(lateralPower, angularPower);

        sleep(20);
    };

    endMotion();
};

void TankChassis::turnToAngle(float angle) {
    motionIsActive = true;
    turnAlgorithm.reset();

    angle = normalize180(angle);

    while (!turnAlgorithm.isSettled() && motionIsActive) {
        float error = angle - imu.heading();
        float power = turnAlgorithm.update(error);

        arcade(0, power);

        sleep(10);
    };

    endMotion();
};

void TankChassis::turnToAngle(float angle, float maxSpeed) {
    float originalMaxSpeed = turnAlgorithm.constants.maxSpeed;
    turnAlgorithm.constants.maxSpeed = maxSpeed;
    turnToAngle(angle);
    turnAlgorithm.constants.maxSpeed = originalMaxSpeed;
};

void TankChassis::leftSwingToAngle(float angle) {
    swingAlgorithm.reset();

    angle = normalize180(angle);

    while (!swingAlgorithm.isSettled()) {
        float error = angle - imu.heading();
        float power = swingAlgorithm.update(error);

        tank(0, -power);

        sleep(10);
    };

    endMotion();
};

void TankChassis::leftSwingToAngle(float angle, float maxSpeed) {
    float originalMaxSpeed = swingAlgorithm.constants.maxSpeed;
    swingAlgorithm.constants.maxSpeed = maxSpeed;
    leftSwingToAngle(angle);
    swingAlgorithm.constants.maxSpeed = originalMaxSpeed;
};

void TankChassis::rightSwingToAngle(float angle) {
    swingAlgorithm.reset();

    angle = normalize180(angle);

    while (!swingAlgorithm.isSettled()) {
        float error = angle - imu.heading();
        float power = swingAlgorithm.update(error);

        tank(power, 0);

        sleep(10);
    };

    endMotion();
};

void TankChassis::rightSwingToAngle(float angle, float maxSpeed) {
    float originalMaxSpeed = swingAlgorithm.constants.maxSpeed;
    swingAlgorithm.constants.maxSpeed = maxSpeed;
    rightSwingToAngle(angle);
    swingAlgorithm.constants.maxSpeed = originalMaxSpeed;
};

void TankChassis::arc(float targetHeading, float leftMultiplier, float rightMultiplier, float maxSpeed) {
    arcAlgorithm.reset();

    const float leftMaxSpeed = maxSpeed * leftMultiplier;
    const float rightMaxSpeed = maxSpeed * rightMultiplier;

    do {
        // calculate error
        float error = normalize180(targetHeading - imu.heading());

        float output = arcAlgorithm.update(error);

        // break if error is less than one
        if (fabs(error) < 1) break;

        tank(CLAMP(output * leftMultiplier, leftMaxSpeed), CLAMP(output * rightMultiplier, rightMaxSpeed));

        sleep(10);
    } while (!arcAlgorithm.isSettled());

    endMotion();
}

void TankChassis::driveToPoint(float x, float y, float backwards) {
    if (!odometry.isConfigured()) return;

    motionIsActive = true;

    driveAlgorithm.reset();
    headingAlgorithm.reset();

    const float startingAngle = odometry.pose.angle(y, x);
    bool crossedPerpendicularLine = false,
         previousCrossedPerpendicularLine = hasCrossedPerpendicularLine(x, y, startingAngle);

    while (!driveAlgorithm.isSettled() && motionIsActive) {
        crossedPerpendicularLine = hasCrossedPerpendicularLine(x, y, startingAngle);
        if (crossedPerpendicularLine && !previousCrossedPerpendicularLine) break;
        previousCrossedPerpendicularLine = crossedPerpendicularLine;

        float lateralError = odometry.pose.distance(x, y);
        float targetAngle(backwards ? (fabs(odometry.pose.angle(x, y)) - 90) : (90 - fabs(odometry.pose.angle(x, y))));
        float headingError = targetAngle - (backwards ? normalize180(odometry.pose.theta + 180) : odometry.pose.theta);
        // float headingError = fabs(odometry.pose.angle(x, y)) - 90 - odometry.pose.theta;

        printf("%f\n", lateralError);

        float headingScaleFactor = cos(DEGREES_TO_RADIANS(headingError));
        float lateralPower = driveAlgorithm.update(lateralError) * headingScaleFactor;
        float angularPower = lateralError < driveAlgorithm.constants.settleError ? 0 : headingAlgorithm.update(normalize90(headingError));

        // prevent moving in the wrong direction
        lateralPower = backwards ? std::fmin(lateralPower, 0) : std::fmax(lateralPower, 0);

        float lateralPowerThreshold = fabs(headingScaleFactor) * driveAlgorithm.constants.maxSpeed;
        lateralPower = CLAMP(lateralPower, lateralPowerThreshold);

        // TODO: minimum drive speed
        // TODO?: speed scaling

        arcade(lateralPower, angularPower);

        sleep(10);
    };

    endMotion();
};

void TankChassis::driveToPose(Pose target, float lead, float setback) {
    if (!odometry.isConfigured()) return;

    motionIsActive = true;
    driveAlgorithm.reset();
    headingAlgorithm.reset();

    bool crossedPerpendicularLine = hasCrossedPerpendicularLine(target.x, target.y, target.theta);
    bool previousCrossedPerpendicularLine = crossedPerpendicularLine;
    bool crossedCenterLine = false;
    bool centerLineSide = hasCrossedCenterLine(target.x, target.y, target.theta);
    bool previousCenterLineSide = centerLineSide;

    const float constantCarrotX = target.x - sin(DEGREES_TO_RADIANS(target.theta));
    const float constantCarrotY = target.y - cos(DEGREES_TO_RADIANS(target.theta));

    Brain.Screen.clearLine(5);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Constant Carrot: (%f, %f)", constantCarrotX, constantCarrotY);

    while (!driveAlgorithm.isSettled() && motionIsActive) {
        crossedPerpendicularLine = hasCrossedPerpendicularLine(target.x, target.y, target.theta);
        if (crossedPerpendicularLine && !previousCrossedPerpendicularLine) break;
        previousCrossedPerpendicularLine = crossedPerpendicularLine;

        centerLineSide = hasCrossedCenterLine(target.x, target.y, target.theta);
        if (centerLineSide != previousCenterLineSide) {
            crossedCenterLine = true;
        };

        float distanceToTarget = odometry.pose.distance(target);

        float carrotX = constantCarrotX * (lead * distanceToTarget + setback);
        float carrotY = constantCarrotY * (lead * distanceToTarget + setback);

        Brain.Screen.clearLine(6);
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Distance To Target: %f", distanceToTarget);
        Brain.Screen.clearLine(7);
        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print("Carrot: (%f, %f)", carrotX, carrotY);

        float driveError = odometry.pose.distance(carrotX, carrotY);
        float headingError = odometry.pose.angle(carrotX, carrotY);

        if (driveError < driveAlgorithm.constants.settleError || crossedCenterLine || driveError < setback) {
            headingError = target.theta - odometry.pose.theta;
            driveError = distanceToTarget;
        };

        float lateralPower = driveAlgorithm.update(driveError);

        float headingScaleFactor = cos(DEGREES_TO_RADIANS(headingError));
        lateralPower *= headingScaleFactor;
        float angularPower = headingAlgorithm.update(normalize90(headingError));

        const float lateralPowerThreshold = fabs(headingScaleFactor) * driveAlgorithm.constants.maxSpeed;
        if (lateralPower > lateralPowerThreshold) {
            lateralPower = driveAlgorithm.constants.maxSpeed;
        } else if (lateralPower < -lateralPowerThreshold) {
            angularPower = -driveAlgorithm.constants.maxSpeed;
        }  // angularPower = CLAMP(angularPower, headingAlgorithm.config.maxSpeed); -> already done by PIDAlgorithm

        // TODO: minimum drive speed
        // TODO?: speed scaling

        arcade(lateralPower, angularPower);

        sleep(10);
    };

    endMotion();
};