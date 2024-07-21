#ifndef PID_H
#define PID_H

#include "chassis.h"
#include "config.h"
#include <map>

enum ManueverType {
    DRIVE,
    TURN,
    SWING,

    // NOT AN ACTUAL MANUEVER - only here for config :)
    HEADING
};

struct PIDConfig {
    float kP, kI, kD, slew, accuracy;
};

class PIDStateManager {
public:
    float kP, kI, kD;

    float previousError = 100000,
          runningIntegral = 0,
          previousMotorPower = 0;

    PIDStateManager(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD){};

    float update(float error);
};

class PIDController {
private:
    Chassis &chassis;

public:
    // Configuration
    std::map<ManueverType, PIDConfig> config = {
        {DRIVE, {DRIVE_PID_kP, DRIVE_PID_kI, DRIVE_PID_kD, DRIVE_PID_ACCURACY}},
        {TURN, {TURN_PID_kP, TURN_PID_kI, TURN_PID_kD, TURN_PID_ACCURACY}},
        {SWING, {SWING_PID_kP, SWING_PID_kI, SWING_PID_kD, SWING_PID_ACCURACY}},
        {HEADING, {HEADING_PID_kP, HEADING_PID_kI, HEADING_PID_kD, 0.1}},
    };

    PIDController(Chassis &chassis) : chassis(chassis){};

    /// @brief  Drives the robot to a target position with Heading PID
    /// @param manueverType - See ManueverType
    /// @param target - Target position
    /// @param maxSpeed - Maximum speed (0 - 100)
    /// @param timeout - Safety timeout in milliseconds
    void manuever(ManueverType manueverType, float target, float maxSpeed, float timeout);
};

#endif  // PID_H
