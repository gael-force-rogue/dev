// #include "vpp/odom.h"
// #include <math.h>

// void Odometry::update(float leftPosition, float rightPosition, float theta) {
//     // Convert parameter to radians and then to inches & calculate deltas
//     leftPosition = DEGREES_TO_RADIANS(leftPosition) * wheelRadius;
//     rightPosition = DEGREES_TO_RADIANS(rightPosition) * wheelRadius;
//     float deltaLeftPosition = leftPosition - previousLeftPosition;
//     float deltaRightPosition = rightPosition - previousRightPosition;
//     previousLeftPosition = leftPosition;
//     previousRightPosition = rightPosition;

//     // Orientation (IMU or Arc)
//     float deltaTheta = theta - pose.theta;

//     // Calculate local translation vector
//     float localX = 0, localY = 0;
//     if (deltaTheta == 0) {
//         localY = (deltaLeftPosition + deltaRightPosition) / 2;
//     } else {
//         // Radius of arc's circle
//         float r = (deltaRightPosition / deltaTheta) + rightOffset;
//         // Chord Length Formula
//         localY = 2 * r * sin(deltaTheta / 2);
//     }

//     // Rotate local translation vector by (startingTheta + deltaTheta / 2)
//     // Convert to polar, rotate, convert back to cartesian
//     float localPolarRadius = 0, localPolarAngle = 0;
//     if (localX == 0 && localY == 0) {
//         localPolarRadius = 0;
//         localPolarAngle = 0;
//     } else {
//         localPolarRadius = sqrt((localX * localX) + (localY * localY));
//         localPolarAngle = atan2(localY, localX);
//     }

//     float globalPolarAngle = localPolarAngle - this->pose.theta - (deltaTheta / 2);

//     float globalCartesianX = localPolarRadius * cos(globalPolarAngle);
//     float globalCartesianY = localPolarRadius * sin(globalPolarAngle);

//     // Update global position
//     pose.x += globalCartesianX;
//     pose.y += globalCartesianY;
//     pose.theta = theta;
// };