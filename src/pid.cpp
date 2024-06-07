#include "pid.h"
#include "config.h"
#include "vex.h"
#include <functional>
#include <iostream>

template <typename T>
T abs(T value) {
    return copysign(value, 1);
}

template <typename T>
T clamp(T value, T minValue, T maxValue) {
    return std::max(std::min(value, maxValue), minValue);
}

void PIDController::manuever(PIDConfig config, float target,
                             float maxSpeed, float timeout) {
    chassis.leftCluster.setDefaultBrakeMode(hold);
    chassis.rightCluster.setDefaultBrakeMode(hold);

    // This section allows this function to work for drive, turn and swing.
    // The 2 points of difference between these maneuvers are the the error source & how the motor power is applied.
    // The lambda takes care of the error source, and the multipliers take care of the motor power.
    float leftMultiplier = 100, rightMultiplier = 100;
    std::function<float()> positionLambda;
    if (config.manueverType == DRIVE) {
        positionLambda = [&]() { return (chassis.leftCluster.averagePosition() + chassis.rightCluster.averagePosition()) / 2; };
    } else {
        positionLambda = [&]() { return chassis.inertialSensor.rotation(); };

        if (config.manueverType == TURN) {
            rightMultiplier = -100;
        } else if (config.manueverType == SWING) {
            if (target > 0) {
                leftMultiplier = 0;
            } else {
                rightMultiplier = 0;
            }
        }
    }

    float currentError = target, previousError = 0, runningIntegral = 0, previousMotorPower = 0;
    timer timeoutTimer;

    while (abs(currentError) >= config.accuracy && timeoutTimer.time(msec) < timeout) {
        float error = target - positionLambda();
        float derivative = error - previousError;
        runningIntegral += error;
        previousError = error;

        float motorPower = (error * config.kP) + (runningIntegral * config.kI) + (derivative * config.kD);
        motorPower = clamp(motorPower, -1.0f, 1.0f);

        // Slew essentially controls how much PID can jump the motor power in a single iteration. E.g. It can't go from 0 to 100 in 1 iteration.
        if (config.slew > 0)
            motorPower = clamp(motorPower, previousMotorPower - config.slew, previousMotorPower + config.slew);

        chassis.leftCluster.spin(motorPower * leftMultiplier);
        chassis.rightCluster.spin(motorPower * rightMultiplier);

        wait(20, msec);
    };

    chassis.leftCluster.stop(coast);
    chassis.rightCluster.stop(coast);
};