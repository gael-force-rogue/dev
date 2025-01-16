#pragma once

#include "vex.h"
#include "vpp/helpers.h"

#include <cmath>

#define DEGREES_TO_RADIANS(v) (v * M_PI / 180.0)
#define RADIANS_TO_DEGREES(v) (v * 180.0 / M_PI)

namespace vpp {
    struct Pose {
        // X, Y, and heading
        float x, y, theta;

        /**
         * @brief Construct a new Pose object
         */
        Pose() : x(0), y(0), theta(0) {};

        /**
         * @brief Construct a new Pose object
         *
         * @param x X coordinate
         * @param y Y coordinate
         * @param theta Heading
         */
        Pose(float x, float y, float theta) : x(x), y(y), theta(theta) {};

        /**
         * @brief Get the distance to a pose
         *
         * @param B Pose B
         * @return Distance between the two poses
         */
        float distance(Pose B);

        /**
         * @brief Calculate the distance to a point
         * @param x X coordinate of the point
         * @param y Y coordinate of the point
         */
        float distance(float x, float y);

        /**
         * @brief Get the angle to a point
         *
         * @param x X coordinate
         * @param y Y coordinate
         * @return float Angle to point
         */
        float angle(float x, float y);
    };

    /**
     * @brief Odometry class
     * @param diameter Diameter of the drivetrain wheels (inches)
     * @param externalRatio External gear ratio of the drivetrain
     */
    class Odometry {
        // Configuration
        float verticalTrackerOffset = 0;
        float verticalTrackerDegreesToInchesConversionFactor = 0;
        vex::rotation verticalTrackerWheel;

        float sidewaysTrackerOffset = 0;
        float sidewaysTrackerDegreesToInchesConversionFactor = 0;
        vex::rotation sidewaysTrackerWheel;

        // Odometry State
        float previousForwardPosition = 0;
        float previousSidewaysPosition = 0;

    public:
        Pose pose;

        Odometry() {
            verticalTrackerWheel = vex::rotation(vex::PORT21, true);
            sidewaysTrackerWheel = vex::rotation(vex::PORT21, true);
        };

        /**
         * @brief Checks if Odometry is usable
         */
        inline bool isConfigured() {
            return verticalTrackerDegreesToInchesConversionFactor != 0;
        };

        /**
         * @brief Resets the pose of the robot
         */
        inline void resetPose() {
            pose = {0, 0, 0};
        };

        /**
         * @brief Adds bare minimum odometry to the chassis
         * @param diameter Diameter of the vertical wheel
         * @param externalRatio External gear ratio of the vertical wheel
         * @param offset Offset of the vertical wheel from the center in inches (positive is to the right & negative is to the left)
         * @param tracker Rotation sensor of the vertical wheel
         */
        inline void withVerticalTrackerWheel(float diameter, float externalRatio, float offset, vex::rotation tracker) {
            this->verticalTrackerOffset = offset;
            this->verticalTrackerDegreesToInchesConversionFactor = calculateDegreesToInchesConversionFactor(diameter, externalRatio);
            this->verticalTrackerWheel = tracker;
        };

        /**
         * @brief Adds bare minimum odometry to the chassis
         * @param diameter Diameter of the vertical wheel
         * @param externalRatio External gear ratio of the vertical wheel
         * @param offset Offset of the vertical wheel from the center in inches (positive is to the right & negative is to the left)
         */
        inline void withVerticalTrackerWheel(float diameter, float externalRatio, float offset) {
            this->verticalTrackerOffset = offset;
            this->verticalTrackerDegreesToInchesConversionFactor = calculateDegreesToInchesConversionFactor(diameter, externalRatio);
        };

        /**
         * @brief Adds sideways tracking
         * @param diameter Diameter of the sideways tracker wheel
         * @param externalRatio External gear ratio of the sideways tracker wheel
         * @param offset Offset of the sideways tracker wheel from the center in inches (positive is to the top & negative is to the bottom)
         */
        inline void withSidewaysTrackerWheel(float diameter, float externalRatio, float offset, vex::rotation tracker) {
            this->sidewaysTrackerOffset = offset;
            this->sidewaysTrackerDegreesToInchesConversionFactor = calculateDegreesToInchesConversionFactor(diameter, externalRatio);
            this->sidewaysTrackerWheel = tracker;
        };

        /**
         * @brief Updates the pose of the robot
         * @param verticalPosition Vertical wheel's position (degrees)
         * @param sidewaysPosition Sideways wheel's position (degrees) - 0 if not using sideways wheel
         * @param rawTheta Raw theta value from the IMU (imu.heading())
         */
        void update(float verticalPosition, float sidewaysPosition, float rawTheta);

        /**
         * @brief Converts a vertical position to inches
         */
        inline float verticalPositionInInches() {
            return verticalTrackerWheel.position(vex::deg) * verticalTrackerDegreesToInchesConversionFactor;
        };

        /**
         * @brief Converts a vertical position to inches
         */
        inline float sidewaysPositionInInches() {
            return sidewaysTrackerWheel.position(vex::deg) * sidewaysTrackerDegreesToInchesConversionFactor;
        };
    };
};  // namespace vpp
