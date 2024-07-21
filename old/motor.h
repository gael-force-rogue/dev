#ifndef MOTOR_H
#define MOTOR_H

#include "vex.h";

class MotorCluster {
private:
    motor &motor1, &motor2, &motor3;

public:
    MotorCluster(motor &motor1, motor &motor2, motor &motor3) : motor1(motor1), motor2(motor2), motor3(motor3){};

    /// @brief Spins the motors with the given velocity
    /// @param velocity From -100 to 100
    void spin(float velocity);
    /// @brief Stops the motors with the given brake mode
    void stop(brakeType mode);
    /// @brief Sets the default brake mode for when spin(0) is called
    void setDefaultBrakeMode(brakeType mode);

    /// @brief Returns the average velocity of the motors. -100 to 100
    float averageVelocity();
    /// @brief Returns the average position of the motor encoders in degrees
    float averagePosition();
    void resetPosition();

    void spinFor(double rotation, double velocity);
};

#endif  // MOTOR_H