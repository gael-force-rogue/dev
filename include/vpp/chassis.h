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

        /**
         * @brief Determines if the robot has crossed a line 1) perpendicular to a line going through robot and target and 2) going through the target point
         * A center line is the line from robot to the target point
         * @param x Target X coordinate
         * @param y Target Y coordinate
         * @param angle Angle of the target point
         * @ref https://www.desmos.com/calculator/zwaytec4zg
         */
        inline bool hasCrossedPerpendicularLine(float x, float y, float angle) {
            return ((y - odometry.pose.y) * cos(DEGREES_TO_RADIANS(angle)) <= -(x - odometry.pose.x) * sin(DEGREES_TO_RADIANS(angle)));
        };

        /**
         * @brief Determines which side the robot is on relative to the center line
         * @note A center line is the line from robot to the target point
         */
        inline bool hasCrossedCenterLine(float x, float y, float targetAngle) {
            return hasCrossedPerpendicularLine(x, y, targetAngle + 90);
        };

        /**
         * @brief Drives to a point
         * @param x X coordinate
         * @param y Y coordinate
         * @param followThrough Whether to stop after driving (used for motion chaining)
         */
        void driveToPoint(float x, float y, bool followThrough = false) {
            PIDAlgorithm driveAlgorithm(driveConstants);
            PIDAlgorithm headingAlgorithm(headingConstants);

            const float startingAngle = atan2(x - odometry.pose.x, y - odometry.pose.y);
            bool crossedPerpendicularLine = false,
                 previousCrossedPerpendicularLine = hasCrossedPerpendicularLine(x, y, startingAngle);

            while (!driveAlgorithm.isSettled()) {
                crossedPerpendicularLine = hasCrossedPerpendicularLine(x, y, startingAngle);
                if (crossedPerpendicularLine && !previousCrossedPerpendicularLine) break;
                previousCrossedPerpendicularLine = crossedPerpendicularLine;

                float lateralError = odometry.pose.distance(x, y);
                float headingError = odometry.pose.angle(x, y);

                float headingScaleFactor = cos(DEGREES_TO_RADIANS(headingError));
                lateralPower = driveAlgorithm.update(lateralError) * headingScaleFactor;
                float angularPower = lateralError < driveConstants.settleError ? 0 : headingAlgorithm.update(normalize90(headingError));

                lateralPower = CLAMP(lateralPower, fabs(heading_scale_factor) * driveConstants.maxSpeed);
                // angularPower = CLAMP(angularPower, headingAlgorithm.config.maxSpeed); -> already done by PIDAlgorithm

                // TODO: minimum drive speed
                // TODO?: speed scaling

                arcade(lateralPower, angularPower);

                sleep(10);
            };

            if (!followThrough) {
                stop(HOLD);
            }
        };

        /**
         * @brief Drives to a pose with a boomerang controller
         * @param target A Pose object representing the target pose
         * @param lead Lead distance
         * @param setback Setback distance
         * @param followThrough Whether to stop after driving (used for motion chaining)
         */
        void driveToPose(Pose target, float lead, float setback, bool followThrough = false) {
            PIDAlgorithm driveAlgorithm(driveConstants);
            PIDAlgorithm headingAlgorithm(headingConstants);

            bool crossedPerpendicularLine = hasCrossedPerpendicularLine(target.x, target.y, target.theta);
            bool previousCrossedPerpendicularLine = crossedPerpendicularLine;
            bool crossedCenterLine = false;
            bool centerLineSide = hasCrossedCenterLine(target.x, target.y, target.theta);
            bool previousCenterLineSide = centerLineSide;

            while (!driveAlgorithm.isSettled()) {
                crossedPerpendicularLine = hasCrossedPerpendicularLine(X_position, Y_position, angle);
                if (crossedPerpendicularLine && !previousCrossedPerpendicularLine) break;
                previousCrossedPerpendicularLine = crossedPerpendicularLine;

                centerLineSide = hasCrossedCenterLine(X_position, Y_position, angle);
                if (centerLineSide != previousCenterLineSide) {
                    crossedCenterLine = true;
                }

                float distanceToTarget = odometry.pose.distance(target);

                float carrotX = x - sin(DEGREES_TO_RADIANS(angle)) * (lead * distanceToTarget + setback);
                float carrotY = y - cos(DEGREES_TO_RADIANS(angle)) * (lead * distanceToTarget + setback);

                float driveError = odometry.pose.distance(carrotX, carrotY);
                float headingError = odometry.pose.angle(carrotX, carrotY);

                if (driveError < driveConstants.settleError || crossedCenterLine || driveError < setback) {
                    headingError = normalize180(angle) - odometry.pose.theta;
                    driveError = distanceToTarget;
                };

                float lateralPower = driveAlgorithm.update(driveError);

                float heading_scale_factor = cos(DEGREES_TO_RADIANS(headingError));
                lateralPower *= heading_scale_factor;
                float angularPower = headingAlgorithm.update(normalize90(headingError));

                lateralPower = CLAMP(lateralPower, fabs(heading_scale_factor) * driveAlgorithm.config.maxSpeed);
                // angularPower = CLAMP(angularPower, headingAlgorithm.config.maxSpeed); -> already done by PIDAlgorithm

                // TODO: minimum drive speed
                // TODO?: speed scaling

                arcade(lateralPower, angularPower);

                sleep(10);
            };

            if (!followThrough) {
                stop(HOLD);
            }
        };

        void turnToPoint(float x, float y, float offset, bool followThrough = false) {
            PIDAlgorithm turnAlgorithm(turnConstants);
            while (!turnAlgorithm.isSettled()) {
                float error = odometry.pose.angle(x, y) + offset;
                float output = turnAlgorithm.update(error);
                arcade(0, output);
                sleep(10);
            };

            if (!followThrough) {
                stop(HOLD);
            }
        }

        void turnToPoint(float x, float y, bool followThrough = false) {
            turnToPoint(x, y, 0, followThrough);
        };
    };
};  // namespace vpp