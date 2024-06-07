#ifndef PID_H
#define PID_H

#include "chassis.h"

enum ManueverType {
    DRIVE,
    TURN,
    SWING
};

struct PIDConfig {
    ManueverType manueverType;
    float kP, kI, kD, slew, accuracy;
};

class PIDController {
  public:
    Chassis chassis;

    PIDController(Chassis &chassis) : chassis(chassis){};

    void manuever(PIDConfig config, float target, float maxSpeed, float timeout);
};

#endif // PID_H
