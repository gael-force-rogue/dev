#pragma once

#include <cmath>

#define DEGREES_TO_RADIANS(degrees) (degrees * M_PI / 180.0)
#define RADIANS_TO_DEGREES(radians) (radians * 180.0 / M_PI)

enum OdometryType {
    IMU_ONLY,
    IMU_WITH_SIDEWAYS
};

enum OdometryConfiguration {
    NO_TRACKER_NO_IMU,
    NO_TRACKER_IMU,
};

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

/// @brief Represents an Odometry algorithm that uses a left and right vertical tracker/encoder and an IMU
class Odometry {
private:
    float leftOffset, rightOffset, wheelRadius;
    float previousLeftPosition = 0, previousRightPosition = 0;

public:
    Pose pose;

    /// @brief Creates an Odometry algorithm
    /// @param config Configuration for the odometry algorithm
    /// @note ALL PARAMETERS MUST BE IN INCHES
    Odometry(float leftOffset, float rightOffset, float wheelRadius)
        : leftOffset(leftOffset), rightOffset(rightOffset), wheelRadius(wheelRadius) {};

    /// @brief Updates the position of the robot
    /// @param leftPosition Left encoder or rotational sensor position (degrees)
    /// @param rightPosition Right encoder or rotational sensor position (degrees)
    /// @param theta The imu.heading() reading (degrees)
    void update(float leftPosition, float rightPosition, float theta);
};