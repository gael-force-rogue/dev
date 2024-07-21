#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"
#include "vex.h"

class Chassis {
  public:
    MotorCluster &leftCluster, &rightCluster;
    inertial &inertialSensor;

    Chassis(MotorCluster &leftCluster, MotorCluster &rightCluster, inertial &inertialSensor);
    void arcade(float lateral, float angular);
    void tank(float left, float right);
};

#endif // CHASSIS_H
