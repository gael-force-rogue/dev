#pragma once

#include "vex.h"

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
         * @brief Get the distance between two poses
         *
         * @param B Pose B
         * @return float Distance between the two poses
         */
        inline float distance(Pose B) const {
            return sqrt(pow(B.x - x, 2) + pow(B.y - y, 2));
        };

        /**
         * @brief Get the angle between two poses
         *
         * @param B Pose B
         * @return float Angle between the two poses
         */
        float angle(Pose B) const {
            return RADIANS_TO_DEGREES(atan2(B.y - y, B.x - x)) - theta;
        };
    };

    class Odometry {
    private:
        float drivetrainDegreesToInchesRatio = 0;

        vex::rotation *verticalRotational = nullptr;
        bool useRightVerticalTracker = false;
        bool useVerticalRotational = false;
        float verticalTrackerOffset;
        float verticalTrackerDegreesToInchesRatio;

        vex::rotation *sidewaysRotational = nullptr;
        bool useSidewaysRotational = false;
        float sidewaysTrackerOffset;
        float sidewaysTrackerDegreesToInchesRatio;

        // Odometry State
        float previousForwardPosition = 0;
        float previousSidewaysPosition = 0;

    public:
        Pose pose;

        Odometry() = default;

        /**
         * @brief Adds bare minimum odometry to the chassis
         * @param diameter Diameter of the drivetrain wheels (inches)
         * @param externalRatio External gear ratio of the drivetrain
         */
        Odometry(float diameter, float externalRatio) {
            this->drivetrainDegreesToInchesRatio = externalRatio / 360.0 * M_PI * diameter;
        };

        /**
         * @brief Adds a vertical tracking wheel to the odometry algorithm
         * @param port Port of the tracking wheel
         * @param diameter Diameter of the tracking wheel (inches)
         * @param offset Offset of the tracking wheel from the center in inches (positive is to the right & negative is to the left)
         * @param gearRatio Gear ratio of the tracking wheel
         */
        inline Odometry &withVerticalTrackerWheel(int port, float diameter, float offset) {
            this->verticalRotational = &vex::rotation(port);
            this->useVerticalRotational = true;
            this->verticalTrackerOffset = offset;
            this->verticalTrackerDegreesToInchesRatio = (diameter * M_PI) / 360;
            return *this;
        };

        /**
         * @brief Adds a sideways tracking wheel to the odometry algorithm
         * @param port Port of the tracking wheel
         * @param diameter Diameter of the tracking wheel (inches)
         * @param offset Offset of the tracking wheel from the center (inches)
         * @param gearRatio Gear ratio of the tracking wheel
         */
        inline Odometry &withSidewaysTrackerWheel(int port, float diameter, float offset) {
            this->sidewaysRotational = &vex::rotation(port);
            this->useSidewaysRotational = true;
            this->sidewaysTrackerOffset = offset;
            this->sidewaysTrackerDegreesToInchesRatio = (diameter * M_PI) / 360;
            return *this;
        };

        /**
         * @brief Resets the pose of the robot
         */
        void resetPose() {
            pose = {0, 0, 0};
        };

        /**
         * @brief Updates the pose of the robot
         * @param drivetrainVerticalPosition Vertical position of the drivetrain
         */
        void update(float drivetrainVerticalPosition, float rawTheta) {
            float rawForward = useVerticalRotational ? (verticalRotational->position(deg) * this->verticalTrackerDegreesToInchesRatio) : (drivetrainVerticalPosition * drivetrainDegreesToInchesRatio);
            float rawSideways = useSidewaysRotational ? sidewaysRotational->position(deg) * this->sidewaysTrackerDegreesToInchesRatio : 0;

            float deltaForward = rawForward - pose.x;
            float deltaSideways = rawSideways - pose.y;
            float thetaRadians = DEGREES_TO_RADIANS(rawTheta);
            float previousThetaRadians = DEGREES_TO_RADIANS(pose.theta);
            float deltaThetaRadians = thetaRadians - previousThetaRadians;
            this->previousForwardPosition = rawForward;
            this->previousSidewaysPosition = rawSideways;
            this->pose.theta = rawTheta;

            float localX = deltaThetaRadians == 0 ? deltaSideways : (2 * sin(deltaThetaRadians / 2)) * ((deltaSideways / deltaThetaRadians) + sidewaysTrackerOffset);
            float localY = deltaThetaRadians == 0 ? deltaForward : (2 * sin(deltaThetaRadians / 2)) * ((deltaForward / deltaThetaRadians) + verticalTrackerOffset);

            bool hasMovedAtAll = localX != 0 || localY != 0;
            float localPolarTheta = hasMovedAtAll ? atan2(localY, localX) : 0;
            float localPolarRadius = hasMovedAtAll ? sqrt(pow(localX, 2) + pow(localY, 2)) : 0;

            float globalPolarTheta = localPolarTheta - previousThetaRadians - (deltaThetaRadians / 2);

            float deltaGlobalX = localPolarRadius * cos(globalPolarTheta);
            float deltaGlobalY = localPolarRadius * sin(globalPolarTheta);

            pose.x += deltaGlobalX;
            pose.y += deltaGlobalY;
        };
    };
};  // namespace vpp
