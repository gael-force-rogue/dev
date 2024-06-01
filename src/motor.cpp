#include "motor.h"
#include "vex.h"

using namespace vex;

MotorCluster::MotorCluster(motor &motor1, motor &motor2, motor &motor3)
    : motor1(motor1), motor2(motor2), motor3(motor3){};

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

void MotorCluster::spin(int velocity) {
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