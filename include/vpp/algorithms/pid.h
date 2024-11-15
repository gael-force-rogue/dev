#pragma once

#include "vex.h"
#include "vpp/helpers.h"
#include <map>
#include <iostream>

namespace vpp {
    struct PIDConstants {
        // Proportional constant
        float kP = 0;
        // Integral constant
        float kI = 0;
        // Derivative constant
        float kD = 0;
        // Maximum speed
        float maxSpeed = 100;
        // Timeout
        float timeout = 1500;
        // Settle error (accuracy)
        float settleError = 1;
        // settle time (must stay settled for this amount of time)
        float settleTime = 300;
        // Maximum velocity (for motion chaining)
        float maxSettleSpeed = 3;

        PIDConstants() = default;
        PIDConstants(float kP, float kI, float kD, float maxSpeed, float timeout, float settleError, float settleTime, float maxSettleSpeed)
            : kP(kP), kI(kI), kD(kD), maxSpeed(maxSpeed), timeout(timeout), settleError(settleError), settleTime(settleTime), maxSettleSpeed(maxSettleSpeed) {};
    };

    class PIDAlgorithm {
    public:
        PIDConstants config;

        float runningIntegral = 0,
              previousMotorPower = 0,
              previousError = 0;

        vex::timer timeoutTimer;
        vex::timer idleTimer;
        float timeSettled = 0;

        PIDAlgorithm(PIDConstants config) : config(config) {
            timeoutTimer.clear();
            idleTimer.clear();
        };

        /// @brief Returns the calculated motor power and updates the state
        /// @param error Current error (target - current position)
        /// @return The calculated motor power (NOT clamped)
        inline float update(float error) {
            runningIntegral += error;
            float derivative = error - previousError;

            // Elimates integral component if oscillating around target
            if ((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) {
                runningIntegral = 0;
            }

            // Checks for settle
            if (fabs(error) < config.settleError) {
                timeSettled += 10;
            } else {
                timeSettled = 0;
            }

            previousError = error;
            return CLAMP((config.kP * error) + (config.kI * runningIntegral) + (config.kD * derivative), config.maxSpeed);
        };

        inline bool isSettled() {
            if (safetyTimer.time() > config.timeout) {
                std::cout << "WARNING: PIDAlgorithm exiting due to safety timeout" << std::endl;
                return true;
            }
            if (timeSettled > config.settleTime) {
                if (fabs(previousMotorPower) >= config.maxSettleSpeed) {
                    std::cout << "WARNING: PIDAlgorithm reached target at " << previousMotorPower << " speed" << std::endl;
                    return true;
                }
            }
            return timeSettled > config.settleTime && fabs(previousMotorPower) < config.maxSettleSpeed;
        };
    };
}  // namespace vpp