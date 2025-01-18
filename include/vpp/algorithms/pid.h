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
        // When to actually start using the integral component
        float iStartError = 1.5;
        // Maximum speed
        float maxSpeed = 100;
        // Timeout
        float timeout = 1500;
        // Settle error (accuracy)
        float settleError = 1;
        // Settle time (must stay settled for this amount of time)
        float settleTime = 300;

        PIDConstants() = default;
        PIDConstants(float kP, float kI, float kD, float iStartError, float maxSpeed, float timeout, float settleError, float settleTime)
            : kP(kP), kI(kI), kD(kD), iStartError(iStartError), maxSpeed(maxSpeed), timeout(timeout), settleError(settleError), settleTime(settleTime) {};
    };

    class PIDAlgorithm {
    public:
        std::string name;
        PIDConstants constants;
        PIDConstants defaultConstants;

        float runningIntegral = 0,
              previousMotorPower = 0,
              previousError = 0;

        vex::timer safetyTimer;
        vex::timer idleTimer;
        float timeSettled = 0;

        PIDAlgorithm() = default;
        PIDAlgorithm(std::string name, PIDConstants constants)
            : name(name), defaultConstants(constants), constants(constants) {};

        /// @brief Resets default constants
        void resetConstants() {
            constants = defaultConstants;
        };

        /**
         * @brief Resets the PIDAlgorithm timers
         */
        inline void reset() {
            safetyTimer.clear();
            idleTimer.clear();
            runningIntegral = 0;
            previousMotorPower = 0;
            previousError = 0;
            timeSettled = 0;
        };

        /// @brief Returns the calculated motor power and updates the state
        /// @param error Current error (target - current position)
        /// @return The calculated motor power (NOT clamped)
        inline float update(float error) {
            // kI only starts having an effect after iStartError (when to speed up)
            if (error < constants.iStartError) {
                runningIntegral += error;
            };
            float derivative = error - previousError;

            // Elimates integral component if oscillating around target
            if ((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) {
                runningIntegral = 0;
            }

            // Checks for settling
            if (fabs(error) < constants.settleError) {
                timeSettled += 10;
            } else {
                timeSettled = 0;
            }

            previousError = error;

            float power = CLAMP((constants.kP * error) + (constants.kI * runningIntegral) + (constants.kD * derivative), constants.maxSpeed);

            return power;
        };

        inline bool isSettled() {
            if (safetyTimer.time() > constants.timeout) {
                // Brain.Screen.setCursor(1, 1);
                // Brain.Screen.print("%s PID: safety timeout\n", this->name.c_str());
                return true;
            };
            return timeSettled > constants.settleTime;
        };
    };
}  // namespace vpp