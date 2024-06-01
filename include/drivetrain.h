#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "motor.h"
#include "vex.h"

enum ManeuverType {
    DRIVE,
    TURN,
    SWING
};

class SmartDrivetrain {
  private:
    MotorCluster &leftCluster, &rightCluster;
    inertial &inertialSensor;

  public:
    SmartDrivetrain(MotorCluster &leftCluster, MotorCluster &rightCluster, inertial &inertialSensor);

    /// @brief Performs a PID maneuver to reach a target position or angle
    /// @param kP Propotional constant
    /// @param kI Integral constant
    /// @param kD Derivative constant
    /// @param slew The maximum change in motor power per iteration
    /// @param accuracy The error within the target that is acceptable
    /// @param maxSpeed The maximum speed of the robot before ending the loop
    /// @param target The target position or angle
    /// @param maneuverType The type of maneuver to perform
    void maneuver(float kP, float kI, float kD,
                  float slew, float accuracy, float maxSpeed,
                  float target, ManeuverType maneuverType);

    void driveForDistance(float distance);
    void turnForAngle(float angle);
    void swingForAngle(float angle);

    void startOdometryLoop();
};

#endif // DRIVETRAIN_H
