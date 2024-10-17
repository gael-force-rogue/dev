#include "motor.h"
#include "vex.h"
#include <iostream>

using namespace vex;

void MotorCluster::stop(brakeType mode) {
    motor1.stop(mode);
    motor2.stop(mode);
    motor3.stop(mode);
}

void MotorCluster::setDefaultBrakeMode(brakeType mode) {
    motor1.setBrake(mode);
    motor2.setBrake(mode);
    motor3.setBrake(mode);
}

void MotorCluster::spin(float velocity) {
    motor1.spin(forward, velocity, percent);
    motor2.spin(forward, velocity, percent);
    motor3.spin(forward, velocity, percent);
}

float MotorCluster::averageVelocity() {
    return (motor1.velocity(percent) + motor2.velocity(percent) + motor3.velocity(percent)) / 3;
}

float MotorCluster::averagePosition() {
    return (motor1.position(degrees) + motor2.position(degrees) + motor3.position(degrees)) / 3;
}

void MotorCluster::resetPosition() {
    motor1.resetPosition();
    motor2.resetPosition();
    motor3.resetPosition();
}

void MotorCluster::spinFor(double rotation, double velocity) {
    std::cout << rotation << std::endl;
    motor1.spinFor(rotation, deg, velocity, velocityUnits::pct, true);
    motor2.spinFor(rotation, deg, velocity, velocityUnits::pct, true);
    motor3.spinFor(rotation, deg, velocity, velocityUnits::pct, true);
}