#pragma once

#include "vex.h"
#include <cmath>

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
        return fabs(previousError) > config.accuracy && (vex::timer::system() - startingTime) < timeout;
    };
};