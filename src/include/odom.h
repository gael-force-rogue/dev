#include "vpp.h"

// constant = M_PI / 180
#define DEGREES_TO_RADIANS(degrees) (degrees * M_PI / 180.0)

enum OdometryType {
    IMU_ONLY,
    IMU_WITH_SIDEWAYS
};

/// @brief Represents an Odometry algorithm that uses a left and right vertical tracker/encoder and an IMU
class Odometry {
private:
    float leftOffset, rightOffset, wheelRadius;
    float previousLeftPosition = 0, previousRightPosition = 0;

public:
    float x = 0, y = 0, theta = 0;

    /// @brief Creates an Odometry algorithm
    /// @param leftOffset The distance from the tracking center to the left wheel
    /// @param rightOffset The distance from the tracking center to the right wheel
    /// @param wheelRadius The radius of the wheels
    /// @note ALL PARAMETERS MUST BE IN INCHES
    Odometry(float leftOffset, float rightOffset, float wheelRadius)
        : leftOffset(leftOffset), rightOffset(rightOffset), wheelRadius(wheelRadius) {};

    /// @brief
    /// @param leftPosition Left encoder or rotational sensor position (degrees)
    /// @param rightPosition Right encoder or rotational sensor position (degrees)
    /// @param theta The imu.heading() reading (degrees)
    void update(float leftPosition, float rightPosition, float theta);
};