#pragma once

#include <map>
#include <iostream>
#include "vpp.h"

#include "config.h"

enum ManueverType {
    DRIVE,
    TURN,
    SWING,
    HEADING
};

struct PIDConfig {
    float kP, kI, kD, accuracy;
};

struct PIDControllerConfig {
    PIDConfig drive, heading, turn, swing;

    PIDConfig operator[](ManueverType manueverType) {
        switch (manueverType) {
            case DRIVE:
                return drive;
            case HEADING:
                return heading;
            case TURN:
                return turn;
            case SWING:
                return swing;
        }
    };
};

class PIDAlgorithm {
public:
    PIDConfig config;

    float runningIntegral = 0,
          previousMotorPower = 0,
          previousError = 0;

    float startingTime, timeout;

    PIDAlgorithm(PIDConfig config, float timeout) : config(config), timeout(timeout) {
        startingTime = vex::timer::system();
    };

    /// @brief Returns the calculated motor power and updates the state
    /// @param error Current error (target - current position)
    /// @return The calculated motor power (NOT clamped)
    inline float update(float error) {
        runningIntegral += error;
        float derivative = error - previousError;
        previousError = error;

        return (config.kP * error) + (config.kI * runningIntegral) + (config.kD * derivative);
    };

    inline bool shouldContinue() {
        return (fabs(previousError) > config.accuracy) && ((vex::timer::system() - startingTime) < timeout);
    };
};

class PIDController {
private:
    vpp::Chassis &chassis;
    vex::inertial &inertialSensor;

public:
    PIDController(vpp::Chassis &chassis, vex::inertial &inertialSensor)
        : chassis(chassis), inertialSensor(inertialSensor) {};

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
    /// @param slew Slew rate
    void drive(float target, float maxSpeed, float timeout, float slew);

    /// @brief Turns to a target angle (degrees)
    /// @param target Target angle (degrees)
    /// @param maxSpeed Maximum speed (0 - 100)
    /// @param timeout Timeout in milliseconds
    void turn(float target, float maxSpeed, float timeout, float slew);

    /// @brief Swings a side of the chassis to a target angle
    /// @param target Target angle (degrees) - positive is left, negative is right
    /// @param maxSpeed Maximum speed (0 - 100)
    /// @param timeout Timeout in milliseconds
    // void swing(float target, float maxSpeed, float timeout);
};
