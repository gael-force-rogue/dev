#ifndef MOTOR_H
#define MOTOR_H

#include "vex.h";

using namespace vex;

class MotorCluster {
  private:
    motor &motor1, &motor2, &motor3;

  public:
    MotorCluster(motor &motor1, motor &motor2, motor &motor3);

    /// @brief Spins the motors with the given velocity
    /// @param velocity From -100 to 100
    void spin(int velocity);
    void stop(brakeType mode);
    void setDefaultBrakeMode(brakeType mode);

    /// @brief Returns the average velocity of the motors. -100 to 100
    float averageVelocity();
    /// @brief Returns the average position of the motor encoders in degrees
    float averagePosition();
    void resetPosition();
};

#endif // MOTOR_H