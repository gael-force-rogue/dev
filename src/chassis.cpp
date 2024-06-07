#include "chassis.h"
#include "motor.h"

void Chassis::arcade(float lateral, float angular) {
    leftCluster.spin(lateral + angular);
    rightCluster.spin(lateral - angular);
};

void Chassis::tank(float left, float right) {
    leftCluster.spin(left);
    rightCluster.spin(right);
};