#pragma once

#include <memory>
#include <vector>

#include "vex.h"

namespace vpp {
    /// @brief Represents the different motor cartridges available for motors.
    /// @ref vex::gearSetting
    enum MotorCartridgeType {
        RED_36 = 0,
        GREEN_18 = 1,
        BLUE_6 = 2,
    };

    /// @brief Represents the different stop modes for motors.
    /// @ref vex::brakeType
    enum MotorStopMode {
        COAST = 0,
        BRAKE = 1,
        HOLD = 2,
    };

    class Motor {
    private:
        vex::motor motor;

    public:
        /// @brief Creates a Motor
        /// @param port Which brain port the motor is connected to
        /// @param motorCartridgeType The cartridge type of the motor
        /// @param reverse Whether the motor is reversed or not
        Motor(int32_t port, MotorCartridgeType motorCartridgeType, bool reverse = false) : motor(vex::motor(port, static_cast<vex::gearSetting>(motorCartridgeType), reverse)){};

        /// @brief Spins the motor.
        /// @param velocity The velocity to spin the motor at ranging from -100 to 100
        inline void spin(float velocity) {
            motor.spin(vex::directionType::fwd, velocity, vex::velocityUnits::pct);
        }

        /// @brief Stops the motor
        /// @param type The stop mode to use
        inline void stop(MotorStopMode type) {
            motor.stop(static_cast<vex::brakeType>(type));
        };

        /// @brief Sets the defualt stop mode of the motor
        /// @param mode
        inline void setStopMode(MotorStopMode mode) {
            motor.setStopping(static_cast<vex::brakeType>(mode));
        };

        /// @brief Returns the current position of the motor in degrees (not clamped)
        /// @return
        inline float position() {
            return motor.position(vex::rotationUnits::deg);
        };

        /// @brief Resets the position of the motor
        inline void resetPosition() {
            motor.resetPosition();
        };
    };

    class MotorGroup {
    private:
        std::vector<std::reference_wrapper<Motor>> motors;
        int motorCount;

    public:
        MotorGroup(std::vector<std::reference_wrapper<Motor>> motors) : motors(motors), motorCount(motors.size()){};

        /// @brief Spins all motors
        /// @param velocity velocity (-100 to 100)
        inline void spin(float velocity) {
            for (auto &motor : motors) {
                motor.get().spin(velocity);
            }
        }

        /// @brief Stops all motors
        /// @param type stop mode
        inline void stop(MotorStopMode mode) {
            for (auto &motor : motors) {
                motor.get().stop(mode);
            }
        }

        /// @brief Sets the default stop mode of the motor group
        /// @param mode
        inline void setStopMode(MotorStopMode mode) {
            for (auto &motor : motors) {
                motor.get().setStopMode(mode);
            }
        };

        /// @brief Returns the average position of all motors in the group
        /// @return Average position
        float averagePosition() {
            float sum = 0;
            for (auto &motor : motors) {
                sum += motor.get().position();
            }
            return sum / motorCount;
        }

        /// @brief Resets the position of all motors in the group
        inline void resetPosition() {
            for (auto &motor : motors) {
                motor.get().resetPosition();
            }
        }
    };
};  // namespace vpp