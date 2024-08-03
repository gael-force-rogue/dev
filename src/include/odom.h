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
    float previousLeftPosition, previousRightPosition;

public:
    float x, y, theta;

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
    void update(float leftPosition, float rightPosition, float theta) {
        // Convert parameter to radians and then to inches & calculate deltas
        leftPosition = DEGREES_TO_RADIANS(leftPosition) * wheelRadius;
        rightPosition = DEGREES_TO_RADIANS(rightPosition) * wheelRadius;
        float deltaLeftPosition = leftPosition - previousLeftPosition;
        float deltaRightPosition = rightPosition - previousRightPosition;
        previousLeftPosition = leftPosition;
        previousRightPosition = rightPosition;

        // Orientation (IMU or Arc)
        float deltaTheta = theta - this->theta;

        // Calculate local translation vector
        float localX = 0, localY;
        if (deltaTheta == 0) {
            localY = (deltaLeftPosition + deltaRightPosition) / 2;
        } else {
            // Radius of arc's circle
            float r = (deltaRightPosition / deltaTheta) + rightOffset;
            // Chord Length Formula
            localY = 2 * r * sin(deltaTheta / 2);
        }

        // Rotate local translation vector by (startingTheta + deltaTheta / 2)
        // Convert to polar, rotate, convert back to cartesian
        float localPolarRadius, localPolarAngle;
        if (localX == 0 && localY == 0) {
            localPolarRadius = 0;
            localPolarAngle = 0;
        } else {
            float localPolarRadius = sqrt(localX * localX + localY * localY);
            float localPolarAngle = atan2(localY, localX);
        }

        float globalPolarAngle = localPolarAngle - this->theta - (deltaTheta / 2);

        float globalCartesianX = localPolarRadius * cos(globalPolarAngle);
        float globalCartesianY = localPolarRadius * sin(globalPolarAngle);

        // Update global position
        x += globalCartesianX;
        y += globalCartesianY;
        this->theta = theta;
    };

    void display(vex::brain &brain) {
        brain.Screen.clearScreen();
        brain.Screen.print("X: %f, Y: %f, Theta: %f", x, y, theta);
    };
};