#include "drivetrain.h"
#include "config.h"
#include "motor.h"
#include <functional>

template <typename T>
T abs(T value) {
    return copysign(value, 1);
}

template <typename T>
T clamp(T value, T minValue, T maxValue) {
    return std::max(std::min(value, maxValue), minValue);
}

class SmartDrivetrain {
  private:
    MotorCluster &leftCluster, &rightCluster;
    inertial &inertialSensor;

  public:
    SmartDrivetrain(MotorCluster &leftCluster, MotorCluster &rightCluster, inertial &inertialSensor)
        : leftCluster(leftCluster), rightCluster(rightCluster), inertialSensor(inertialSensor){};

    // Basic PID
    void maneuver(float kP, float kI, float kD,
                  float slew, float accuracy, float maxSpeed,
                  float target, ManeuverType maneuverType) {
        float error = accuracy, previousError = 0;
        float integral = 0, derivative = 0;
        float motorPower = 0, previousMotorPower = 0;

        float leftMultiplier = 1, rightMultiplier = 1;
        std::function<float()> positionLambda;
        if (maneuverType == DRIVE) {
            positionLambda = [&]() { return (leftCluster.averagePosition() + rightCluster.averagePosition()) / 2; };
        } else {
            positionLambda = [&]() { return inertialSensor.rotation(); };

            if (maneuverType == TURN) {
                rightMultiplier = -1;
            } else if (maneuverType == SWING) {
                if (target > 0) {
                    leftMultiplier = 0;
                } else {
                    rightMultiplier = 0;
                }
            } else {
                throw "Invalid maneuver type!";
            }
        }
        leftMultiplier *= 100;
        rightMultiplier *= 100;

        leftCluster.setDefaultBrakeMode(hold);
        rightCluster.setDefaultBrakeMode(hold);

        // Keep going until accuracy reached and robot is not going too fast
        while (abs(error) > accuracy ||
               abs(leftCluster.averageVelocity()) > maxSpeed || abs(rightCluster.averageVelocity()) > maxSpeed) {
            error = target - positionLambda();
            integral += error;
            derivative = error - previousError;
            previousError = error;

            // Calculate motor power & clamp to [-1, 1]
            motorPower = (kP * error) + (kI * integral) + (kD * derivative);
            motorPower = clamp(motorPower, -1.0f, 1.0f);

            // Slew rate limiter
            if (slew != 0)
                motorPower = clamp(motorPower, previousMotorPower - slew, previousMotorPower + slew);

            // Perform the actual movement
            leftCluster.spin(motorPower * leftMultiplier);
            rightCluster.spin(motorPower * rightMultiplier);

            task::sleep(10);
        }

        leftCluster.spin(0);
        rightCluster.spin(0);

        task::sleep(500);
    };

    void driveForDistance(float distance) {
        maneuver(DRIVE_PID_kP, DRIVE_PID_kI, DRIVE_PID_kD,
                 DRIVE_PID_SLEW, DRIVE_PID_ACCURACY, DRIVE_PID_MAX_SPEED,
                 distance, DRIVE);
    };

    void turnForAngle(float angle) {
        maneuver(TURN_PID_kP, TURN_PID_kI, TURN_PID_kD,
                 TURN_PID_SLEW, TURN_PID_ACCURACY, TURN_PID_MAX_SPEED,
                 angle, TURN);
    };

    void swingForAngle(float angle) {
        maneuver(SWING_PID_kP, SWING_PID_kI, SWING_PID_kD,
                 SWING_PID_SLEW, SWING_PID_ACCURACY, SWING_PID_MAX_SPEED,
                 angle, SWING);
    };
};