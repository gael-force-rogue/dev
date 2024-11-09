#include "chassis.h"
#include "motor.h"
#include <iostream>

Chassis::Chassis(MotorCluster &leftCluster, MotorCluster &rightCluster, inertial &inertialSensor)
    : leftCluster(leftCluster), rightCluster(rightCluster), inertialSensor(inertialSensor){};

void Chassis::arcade(float lateral, float angular) {
    leftCluster.spin(lateral + angular);
    rightCluster.spin(lateral - angular);
};

void Chassis::tank(float left, float right) {
    leftCluster.spin(left);
    rightCluster.spin(right);
};