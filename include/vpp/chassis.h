#pragma once

#include "vex.h"

#include "vpp/devices/motor.h"
#include "vpp/devices/imu.h"
#include "vpp/devices/rotation.h"
#include "vpp/algorithms/pid.h"
#include "vpp/algorithms/odometry.h"
#include "helpers.h"
#include <algorithm>
#include <iostream>

namespace vpp {
    class TankChassis {
    private:
        PIDConstants defaultDriveConstants;
        PIDConstants defaultHeadingConstants;
        PIDConstants defaultTurnConstants;
        PIDConstants defaultSwingConstants;

    public:
        MotorGroup &leftGroup;
        MotorGroup &rightGroup;
        IMU &imu;

        PIDConstants driveConstants;
        PIDConstants headingConstants;
        PIDConstants turnConstants;
        PIDConstants swingConstants;

        Odometry odometry;

        /**
         * @brief Constructs a new TankChassis object
         * @param left Left motor group
         * @param right Right motor group
         */
        TankChassis(MotorGroup &left, MotorGroup &right, IMU &imu) : leftGroup(leftGroup), rightGroup(rightGroup), imu(imu) {};
        /**
         * @brief Sets the default drive constants
         * @param kP Proportional constant
         * @param kI Integral constant
         * @param kD Derivative constant
         * @param maxSpeed Maximum speed
         * @param timeout Timeout
         * @param settleError Settle error
         * @param settleTime Settle time
         * @param maxSettleSpeed Maximum settle speed
         * @return TankChassis*
         */
        inline TankChassis &withDriveConstants(float kP, float kI, float kD, float maxSpeed, float timeout, float settleError, float settleTime, float maxSettleSpeed) {
            this->defaultDriveConstants = {kP, kI, kD, maxSpeed, timeout, settleError, settleTime, maxSettleSpeed};
            this->driveConstants = this->defaultDriveConstants;
            return *this;
        };

        /**
         * @brief Sets the default heading constants
         */
        inline TankChassis &withHeadingConstants(float kP, float kI, float kD, float maxSpeed, float timeout, float settleError, float settleTime, float maxSettleSpeed) {
            this->defaultHeadingConstants = {kP, kI, kD, maxSpeed, timeout, settleError, settleTime, maxSettleSpeed};
            this->headingConstants = this->defaultHeadingConstants;
            return *this;
        };

        /**
         * @brief Sets the default turn constants
         */
        inline TankChassis &withTurnConstants(float kP, float kI, float kD, float maxSpeed, float timeout, float settleError, float settleTime, float maxSettleSpeed) {
            this->defaultTurnConstants = {kP, kI, kD, maxSpeed, timeout, settleError, settleTime, maxSettleSpeed};
            this->turnConstants = this->defaultTurnConstants;
            return *this;
        };

        /**
         * @brief Sets the default swing constants
         */
        inline TankChassis &withSwingConstants(float kP, float kI, float kD, float maxSpeed, float timeout, float settleError, float settleTime, float maxSettleSpeed) {
            this->defaultSwingConstants = {kP, kI, kD, maxSpeed, timeout, settleError, settleTime, maxSettleSpeed};
            this->swingConstants = this->defaultSwingConstants;
            return *this;
        };

        inline TankChassis &withOdometry(Odometry odometry) {
            this->odometry = odometry;
        };

        /**
         * @brief Resets the drive constants in case they are changed mid-run
         */
        void resetToInitialConstants();

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
        };

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
         * @brief Stops the chassis
         * @param mode COAST, BRAKE, or HOLD
         */
        inline void stop(MotorStopMode mode) {
            leftGroup.stop(mode);
            rightGroup.stop(mode);
        };

        /**
         * @brief Drives the chassis a certain distance
         * @param distance Distance to drive
         * @param followThrough Whether to stop after driving (used for motion chaining)
         */
        void driveDistance(float distance, bool followThrough = false);

        /**
         * @brief Drives the chassis a certain distance with temporary exit conditions
         * @param distance Distance to drive
         * @param maxSpeed Maximum speed
         * @param followThrough Whether to stop after driving (used for motion chaining)
         */
        void driveDistance(float distance, float maxSpeed, bool followThrough = false);

        /**
         * @brief Turns the chassis to a certain angle
         * @param angle Angle to turn to (will be normalized)
         * @param followThrough Whether to stop after turning (used for motion chaining)
         */
        void turnToAngle(float angle, bool followThrough = false);

        /**
         * @brief Turns the chassis to a certain angle with temporary exit conditions
         * @param angle Angle to turn to (will be normalized)
         * @param maxSpeed Maximum speed
         * @param followThrough Whether to stop after turning (used for motion chaining)
         */
        void turnToAngle(float angle, float maxSpeed, bool followThrough = false);

        /**
         * @brief Swings the chassis using only the right motors to a certain angle
         * @param angle Angle to swing to (will be normalized)
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void leftSwingToAngle(float angle, bool followThrough = false);

        /**
         * @brief Swings the chassis using only the right motors to a certain angle with temporary exit conditions
         * @param angle Angle to swing to (will be normalized)
         * @param maxSpeed Maximum speed
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void leftSwingToAngle(float angle, float maxSpeed, bool followThrough = false);

        /**
         * @brief Swings the chassis using only the left motors to a certain angle
         * @param angle Angle to swing to (will be normalized)
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void rightSwingToAngle(float angle, bool followThrough = false);

        /**
         * @brief Swings the chassis using only the left motors to a certain angle with temporary exit conditions
         * @param angle Angle to swing to (will be normalized)
         * @param maxSpeed Maximum speed
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void rightSwingToAngle(float angle, float maxSpeed, bool followThrough = false);
    };
};  // namespace vpp