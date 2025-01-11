#include "vpp/algorithms/odometry.h"

#include "vex.h"

using namespace vex;

float vpp::Pose::distance(Pose B) {
    return sqrt(pow(B.x - x, 2) + pow(B.y - y, 2));
};

float vpp::Pose::distance(float x, float y) {
    // Can also be written as: ;
    return sqrt(pow(x - this->x, 2) + pow(y - this->y, 2));
    //  return hypot(x - this->x, y - this->y);
};

float vpp::Pose::angle(float x, float y) {
    return RADIANS_TO_DEGREES(atan2(y - this->y, x - this->x));
};

void vpp::Odometry::update(float verticalPosition, float sidewaysPosition, float rawTheta) {
    float rawForward = verticalPosition * verticalTrackerDegreesToInchesConversionFactor;
    float rawSideways = sidewaysPosition * sidewaysTrackerDegreesToInchesConversionFactor;

    float deltaForward = rawForward - previousForwardPosition;
    float deltaSideways = rawSideways - previousSidewaysPosition;
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