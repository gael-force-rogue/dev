#include "pid.h"
#include <vex.h>

/// @brief  Normalizes an angle to be between 0 and 180
/// @param angle Angle to normalize
/// @return Normalized angle
float normalize180(float angle) {
    while (angle < -180) {
        angle += 360;
    }
    while (angle >= 180) {
        angle -= 360;
    }
    return angle;
};

void PIDController::drive(float target, float maxSpeed, float timeout, float slew) {
    PIDAlgorithm algorithm(config.drive, timeout);
    PIDAlgorithm headingAlgorithm(config.heading, timeout);

    const float leftStartingPosition = chassis.leftGroup.averagePosition();
    const float rightStartingPosition = chassis.rightGroup.averagePosition();
    const float startingAngle = inertialSensor.heading();

    std::cout << "Starting Angle: " << startingAngle << std::endl;

    float previousLateralMotorPower = 0;

    do {
        float lateralError = target - ((chassis.leftGroup.averagePosition() - leftStartingPosition + chassis.rightGroup.averagePosition() - rightStartingPosition) / 2);
        float angularError = startingAngle - normalize180(inertialSensor.heading());

        std::cout << "Lateral Error: " << lateralError << std::endl;

        float lateralMotorPower = algorithm.update(lateralError);
        float angularMotorPower = headingAlgorithm.update(angularError);

        if (lateralMotorPower > previousLateralMotorPower + slew)
            lateralMotorPower = previousLateralMotorPower + slew;
        else if (lateralMotorPower < previousLateralMotorPower - slew)
            lateralMotorPower = previousLateralMotorPower - slew;

        chassis.arcade(lateralMotorPower, angularMotorPower, maxSpeed);

        previousLateralMotorPower = lateralMotorPower;
        vpp::sleep(20);
    } while (algorithm.shouldContinue());

    chassis.arcade(0, 0);
};

void PIDController::turn(float target, float maxSpeed, float timeout, float slew) {
    PIDAlgorithm algorithm(config.turn, timeout);

    // const float startingAngle = normalize180(inertialSensor.heading());
    const float startingAngle = inertialSensor.heading();

    do {
        float error = normalize180(target - (inertialSensor.heading() - startingAngle));
        float motorPower = algorithm.update(error);

        std::cout << "Error: " << error << std::endl;

        chassis.arcade(0, motorPower, maxSpeed);

        vpp::sleep(20);
    } while (algorithm.shouldContinue());

    chassis.arcade(0, 0);
};
