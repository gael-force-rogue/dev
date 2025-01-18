#pragma once

#include "vex.h"

#include "vpp/devices/motor.h"
#include "vpp/devices/imu.h"
#include "vpp/algorithms/pid.h"
#include "vpp/algorithms/odometry.h"
#include "helpers.h"
#include <algorithm>
#include <iostream>

#define ASYNC(action) vex::thread([&]() { action; });

#define USE_CHASSIS_CONSTANTS                                    \
    auto driveConstants = &chassis.driveAlgorithm.constants;     \
    auto headingConstants = &chassis.headingAlgorithm.constants; \
    auto turnConstants = &chassis.turnAlgorithm.constants;       \
    auto swingConstants = &chassis.swingAlgorithm.constants;     \
    auto arcConstants = &chassis.arcAlgorithm.constants;

#define START_CHAIN \
    chassis.motionIsChained = true;

#define END_CHAIN                    \
    chassis.motionIsChained = false; \
    chassis.endMotion();             \
    chassis.stop(HOLD);

namespace vpp {
    enum TankChassisMotion {
        DRIVE,
        TURN,
        SWING,
        ARC
    };

    class TankChassis {
    private:
        bool motionIsActive = false;

        float inchesToDegreesConversionFactor;
        float degreesToInchesConversionFactor;

    public:
        bool motionIsChained = false;

        MotorGroup &leftGroup;
        MotorGroup &rightGroup;
        IMU &imu;

        Odometry odometry;

        PIDAlgorithm driveAlgorithm;
        PIDAlgorithm headingAlgorithm;
        PIDAlgorithm turnAlgorithm;
        PIDAlgorithm swingAlgorithm;
        PIDAlgorithm arcAlgorithm;

        /**
         * @brief Constructs a new TankChassis object
         * @param left Left MotorGroup
         * @param right Right MotorGroup
         * @param imu IMU
         * @param diameter Diameter of the drivetrain wheels in inches
         * @param externalRatio External gear ratio of the drivetrain (motor:wheel)
         * @param offset Offset of the vertical tracking wheel from the center in inches (right is positive & left is negative)
         */
        TankChassis(MotorGroup &left, MotorGroup &right, IMU &imu,
                    float diameter, float externalRatio, float offset)
            : leftGroup(left), rightGroup(right), imu(imu) {
            this->inchesToDegreesConversionFactor = (360 * externalRatio) / (diameter * M_PI);
            this->degreesToInchesConversionFactor = (diameter * M_PI) / (360 * externalRatio);
            this->odometry.withVerticalTrackerWheel(diameter, externalRatio, offset);
        };

        /**
         * @brief Sets the default drive constants
         * @param kP Proportional constant
         * @param kI Integral constant
         * @param kD Derivative constant
         * @param iStartError Integral start error
         * @param maxSpeed Maximum speed
         * @param timeout Timeout
         * @param settleError Settle error
         * @param settleTime Settle time
         * @return TankChassis*
         */
        inline TankChassis &withDriveConstants(float kP, float kI, float kD, float iStartError, float maxSpeed, float timeout, float settleError, float settleTime) {
            this->driveAlgorithm = PIDAlgorithm("drive", {kP, kI, kD, iStartError, maxSpeed, timeout, settleError, settleTime});
            return *this;
        };

        /**
         * @brief Sets the default heading constants
         * @param kP Proportional constant
         * @param kI Integral constant
         * @param kD Derivative constant
         * @param iStartError Integral start error
         * @param maxSpeed Maximum speed
         * @param timeout Timeout
         * @param settleError Settle error
         * @param settleTime Settle time
         * @return TankChassis*
         */
        inline TankChassis &withHeadingConstants(float kP, float kI, float kD, float iStartError, float maxSpeed, float timeout, float settleError, float settleTime) {
            this->headingAlgorithm = PIDAlgorithm("heading", {kP, kI, kD, iStartError, maxSpeed, timeout, settleError, settleTime});
            return *this;
        };

        /**
         * @brief Sets the default turn constants
         * @param kP Proportional constant
         * @param kI Integral constant
         * @param kD Derivative constant
         * @param iStartError Integral start error
         * @param maxSpeed Maximum speed
         * @param timeout Timeout
         * @param settleError Settle error
         * @param settleTime Settle time
         * @return TankChassis*
         */
        inline TankChassis &withTurnConstants(float kP, float kI, float kD, float iStartError, float maxSpeed, float timeout, float settleError, float settleTime) {
            this->turnAlgorithm = PIDAlgorithm("turn", {kP, kI, kD, iStartError, maxSpeed, timeout, settleError, settleTime});
            return *this;
        };

        /**
         * @brief Sets the default swing constants
         * @param kP Proportional constant
         * @param kI Integral constant
         * @param kD Derivative constant
         * @param iStartError Integral start error
         * @param maxSpeed Maximum speed
         * @param timeout Timeout
         * @param settleError Settle error
         * @param settleTime Settle time
         * @return TankChassis*
         */
        inline TankChassis &withSwingConstants(float kP, float kI, float kD, float iStartError, float maxSpeed, float timeout, float settleError, float settleTime) {
            this->swingAlgorithm = PIDAlgorithm("swing", {kP, kI, kD, iStartError, maxSpeed, timeout, settleError, settleTime});
            return *this;
        };

        /**
         * @brief Sets the default arc constants
         * @param kP Proportional constant
         * @param kI Integral constant
         * @param kD Derivative constant
         * @param iStartError Integral start error
         * @param maxSpeed Maximum speed
         * @param timeout Timeout
         * @param settleError Settle error
         * @param settleTime Settle time
         * @return TankChassis*
         */
        inline TankChassis &withArcConstants(float kP, float kI, float kD, float iStartError, float maxSpeed, float timeout, float settleError, float settleTime) {
            this->arcAlgorithm = PIDAlgorithm("arc", {kP, kI, kD, iStartError, maxSpeed, timeout, settleError, settleTime});
            return *this;
        };

        /**
         * @brief Configures a dedicated vertical tracking wheel for odometry (fallbacks to drivetrain)
         * @param diameter Diameter of the vertical tracking wheel in inches
         * @param externalRatio External gear ratio of the vertical tracking wheel (tracker:wheel)
         * @param offset Offset of the vertical tracking wheel from the center in inches (right is positive & left is negative)
         * @param port Port of the tracking wheel
         * @param reverse Whether the tracking wheel is reversed
         * @return TankChassis*
         */
        inline TankChassis &withVerticalTrackerWheel(float diameter, float externalRatio, float offset, vex::rotation tracker) {
            this->odometry.withVerticalTrackerWheel(diameter, externalRatio, offset, tracker);
            return *this;
        };

        /**
         * @brief Configures a sideways tracking wheel for odometry
         * @param diameter Diameter of the sideways tracker wheel
         * @param externalRatio External gear ratio of the sideways tracker wheel
         * @param offset Offset of the sideways tracker wheel from the center in inches (positive is to the top & negative is to the bottom)
         */
        inline TankChassis &withSidewaysTrackerWheel(float diameter, float externalRatio, float offset, vex::rotation tracker) {
            this->odometry.withSidewaysTrackerWheel(diameter, externalRatio, offset, tracker);
            return *this;
        };

        /**
         * @brief Resets all constants
         */
        void resetConstants();

        /**
         * @brief Calibrates the IMU, resets the motor positions, and resets the Odometry pose
         */
        void calibrate();

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
         * @brief Stops the chassis with the default stop mode
         * @param mode COAST, BRAKE, or HOLD
         */
        inline void stop() {
            leftGroup.stop();
            rightGroup.stop();
        };

        /// BASIC MOTIONS

        inline void endMotion() {
            motionIsActive = false;
            stop(HOLD);
        };

        /**
         * @brief Drives the chassis a certain distance
         * @param distance Distance to drive
         * @param followThrough Whether to stop after driving (used for motion chaining)
         */
        void driveDistance(float distance, float earlyExitError = 0);

        /**
         * @brief Turns the chassis to a certain angle
         * @param angle Angle to turn to (will be normalized)
         * @param followThrough Whether to stop after turning (used for motion chaining)
         */
        void turnToAngle(float angle);

        /**
         * @brief Turns the chassis to a certain angle with temporary exit conditions
         * @param angle Angle to turn to (will be normalized)
         * @param maxSpeed Maximum speed
         * @param followThrough Whether to stop after turning (used for motion chaining)
         */
        void turnToAngle(float angle, float maxSpeed);

        /**
         * @brief Swings the chassis using only the right motors to a certain angle
         * @param angle Angle to swing to (will be normalized)
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void leftSwingToAngle(float angle);

        /**
         * @brief Swings the chassis using only the right motors to a certain angle with temporary exit conditions
         * @param angle Angle to swing to (will be normalized)
         * @param maxSpeed Maximum speed
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void leftSwingToAngle(float angle, float maxSpeed);

        /**
         * @brief Swings the chassis using only the left motors to a certain angle
         * @param angle Angle to swing to (will be normalized)
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void rightSwingToAngle(float angle);

        /**
         * @brief Swings the chassis using only the left motors to a certain angle with temporary exit conditions
         * @param angle Angle to swing to (will be normalized)
         * @param maxSpeed Maximum speed
         * @param followThrough Whether to stop after swinging (used for motion chaining)
         */
        void rightSwingToAngle(float angle, float maxSpeed);

        /// ODOMETRY-BASED MOTIONS

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
        void driveToPoint(float x, float y, float backwards);

        /**
         * @brief Drives to a pose with a boomerang controller
         * @param target A Pose object representing the target pose
         * @param lead Lead distance
         * @param setback Setback distance
         * @param followThrough Whether to stop after driving (used for motion chaining)
         */
        void driveToPose(Pose target, float lead, float setback);

        /**
         * @brief Turns the chassis to a point
         * @param x X coordinate
         * @param y Y coordinate
         * @param offset Offset angle
         * @param followThrough Whether to stop after turning (used for motion chaining)
         */
        void turnToPoint(float x, float y, float offset) {
            turnAlgorithm.reset();

            while (!turnAlgorithm.isSettled()) {
                float error = odometry.pose.angle(x, y) + offset;
                float output = turnAlgorithm.update(error);
                arcade(0, output);
                sleep(10);
            };

            endMotion();
        }

        /**
         * @brief Turns the chassis to a point
         * @param x X coordinate
         * @param y Y coordinate
         * @param followThrough Whether to stop after turning (used for motion chaining)
         */
        void turnToPoint(float x, float y) {
            turnToPoint(x, y, 0);
        };

        /**
         * @brief does a circular arc to target heading using a ratio on the PID outputs.
         *
         * @param heading target angle
         * @param leftMult 0< leftMult <1 ratio of left side pid output
         * @param rightMult 0<rightMult<1 ratio of right side pid output
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         *
         */
        void arc(float targetHeading, float leftMultiplier, float rightMultiplier, float maxSpeed);
    };
};  // namespace vpp