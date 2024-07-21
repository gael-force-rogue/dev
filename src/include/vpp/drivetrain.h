#pragma once

#include "vex.h"
#include "vpp/motor.h"
#include <algorithm>

namespace vpp {
    class Drivetrain {
    public:
        MotorGroup &leftGroup, &rightGroup;

        Drivetrain(MotorGroup &left, MotorGroup &right) : leftGroup(left), rightGroup(right){};

        /// @brief Drives with arcade controls
        /// @param lateral Lateral input
        /// @param angular Angular input
        /// @ref https://wiki.purduesigbots.com/software/robotics-basics/arcade-drive
        inline void arcade(float lateral, float angular) {
            leftGroup.spin(lateral + angular);
            rightGroup.spin(lateral - angular);
        };

        /// @brief Drives with tank controls
        /// @param left Left motor group speed
        /// @param right Right motor group speed
        inline void tank(float left, float right) {
            leftGroup.spin(left);
            rightGroup.spin(right);
        };

        /// @brief Sets the default brake mode for both motor groups
        /// @param mode COAST, BRAKE, or HOLD
        inline void setStopMode(MotorStopMode mode) {
            leftGroup.setStopMode(mode);
            rightGroup.setStopMode(mode);
        };
    };
};  // namespace vpp