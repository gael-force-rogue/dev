#pragma once

#include <map>
#include <iostream>
#include "config.h"
#include "vpp/chassis.h"

class PIDController {
private:
    vpp::Chassis &chassis;
    vex::inertial &inertialSensor;

public:
    PIDController(vpp::Chassis &chassis, vex::inertial &inertialSensor) : chassis(chassis), inertialSensor(inertialSensor) {};

    PIDControllerConfig config{
        {DRIVE_PID_kP, DRIVE_PID_kI, DRIVE_PID_kD, DRIVE_PID_ACCURACY},
        {HEADING_PID_kP, HEADING_PID_kI, HEADING_PID_kD, HEADING_PID_ACCURACY},
        {TURN_PID_kP, TURN_PID_kI, TURN_PID_kD, TURN_PID_ACCURACY},
        {SWING_PID_kP, SWING_PID_kI, SWING_PID_kD, SWING_PID_ACCURACY},
    };

    /// @brief Drives in a straight line to a target position
    /// @param target Target position
    /// @param maxSpeed Maximum speed (0 - 100)
    /// @param timeout Timeout in milliseconds
    void drive(float target, float maxSpeed, float timeout);

    /// @brief Turns to a target angle (degrees)
    /// @param target Target angle (degrees)
    /// @param maxSpeed Maximum speed (0 - 100)
    /// @param timeout Timeout in milliseconds
    void turn(float target, float maxSpeed, float timeout);

    /// @brief Swings a side of the chassis to a target angle
    /// @param target Target angle (degrees) - positive is left, negative is right
    /// @param maxSpeed Maximum speed (0 - 100)
    /// @param timeout Timeout in milliseconds
    void swing(float target, float maxSpeed, float timeout);
};
