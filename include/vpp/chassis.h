#pragma once

#include "vex.h"
#include "vpp/motor.h"
#include "vpp/odom.h"
#include "helpers.h"
#include "config.h"
#include <algorithm>
#include <iostream>

namespace vpp {
    class Chassis {
    public:
        MotorGroup &leftGroup;
        MotorGroup &rightGroup;

        /**
         * @brief Constructs a new Chassis object
         * @param left Left motor group
         * @param right Right motor group
         */
        Chassis(MotorGroup &left, MotorGroup &right)
            : leftGroup(left), rightGroup(right) {};

        /**
         * @brief Drives with arcade controls
         * @param lateral Lateral input
         * @param angular Angular input
         * @ref https://wiki.purduesigbots.com/software/robotics-basics/arcade-drive
         */
        inline void arcade(float lateral, float angular) {
            leftGroup.spin(lateral + angular);
            rightGroup.spin(lateral - angular);
        };

        /**
         * @brief Drives with arcade controls
         * @param lateral Lateral input
         * @param angular Angular input
         * @param maxSpeed Maximum speed
         * @ref https://wiki.purduesigbots.com/software/robotics-basics/arcade-drive
         */
        inline void arcade(float lateral, float angular, float maxSpeed) {
            tank(CLAMP(lateral + angular, maxSpeed), CLAMP(lateral - angular, maxSpeed));
        }

        /**
         * @brief Drives with tank controls
         * @param left Left input
         * @param right Right input
         * @ref https://wiki.purduesigbots.com/software/robotics-basics/tank-drive
         */
        inline void tank(float left, float right) {
            leftGroup.spin(left);
            rightGroup.spin(right);
        };

        /**
         * @brief Sets the default brake mode for both motor groups
         * @param mode COAST, BRAKE, or HOLD
         */
        inline void setDefaultStopMode(MotorStopMode mode) {
            leftGroup.setDefaultStopMode(mode);
            rightGroup.setDefaultStopMode(mode);
        };

        /**
         * @brief Goes to Point B
         * @param target Target point
         * @param maxSpeed Maximum speed
         * @param timeout Timeout in milliseconds
         */
        // void moveToPoint(Pose target, float maxSpeed, float timeout);

        /**
         * @brief Turns to a point
         * @param target Target point
         * @param maxSpeed Maximum speed
         * @param timeout Timeout in milliseconds
         */
        // void turnToPoint(Pose target, float maxSpeed, float timeout);
    };
};  // namespace vpp