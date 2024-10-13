#pragma once
#include "vex.h"
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

// vpp.h
namespace vpp {
    /// @brief Sleeps the current thread for a specified amount of time
    /// @param time Time to wait in milliseconds
    inline void sleep(int time) { vex::this_thread::sleep_for(time); };
    const int PORT1 = vex::PORT1;
    const int PORT2 = vex::PORT2;
    const int PORT3 = vex::PORT3;
    const int PORT4 = vex::PORT4;
    const int PORT5 = vex::PORT5;
    const int PORT6 = vex::PORT6;
    const int PORT7 = vex::PORT7;
    const int PORT8 = vex::PORT8;
    const int PORT9 = vex::PORT9;
    const int PORT10 = vex::PORT10;
    const int PORT11 = vex::PORT11;
    const int PORT12 = vex::PORT12;
    const int PORT13 = vex::PORT13;
    const int PORT14 = vex::PORT14;
    const int PORT15 = vex::PORT15;
    const int PORT16 = vex::PORT16;
    const int PORT17 = vex::PORT17;
    const int PORT18 = vex::PORT18;
    const int PORT19 = vex::PORT19;
    const int PORT20 = vex::PORT20;
    const int PORT21 = vex::PORT21;
};  // namespace vpp

// tuner.h
namespace vpp {
    enum ConfigTunerAction {
        RUN,
        STOP,
        CHANGED,
        NOTHING
    };
    class ConfigTuner {
    private:
        Controller &controller;
        std::vector<std::string> keys;
        std::vector<float> values;
        int currentIndex = 0, maxIndex;
        float incrementBy, deadzone;

    public:
        /// @brief hi
        /// @param controller
        /// @param keys
        /// @param values
        /// @param incrementBy
        /// @param deadzone
        ConfigTuner(Controller &controller, std::vector<std::string> keys, std::vector<float> values,
                    float incrementBy = 0.1, float deadzone = 20);
        void render();
        ConfigTunerAction checkForAction();
    };
}  // namespace vpp

// imu.h
namespace vpp {
    class IMU {
    private:
        vex::inertial inertial;

    public:
        IMU();
        float roll();
        float pitch();
        float yaw();
    };
}  // namespace vpp

// motor.h
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
        Motor(int32_t port, MotorCartridgeType motorCartridgeType, bool reverse = false) : motor(vex::motor(port, static_cast<vex::gearSetting>(motorCartridgeType), reverse)) {};
        /// @brief Creates a Motor with a blue 6:1 cartridge
        /// @param port Which brain port the motor is connected to
        /// @param reverse Whether the motor is reversed or not
        Motor(int32_t port, bool reverse = false) : motor(vex::motor(port, vex::gearSetting::ratio6_1, reverse)) {};
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
        /// @brief Stops the motor with the default stop mode
        inline void stop() {
            motor.stop();
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
        MotorGroup(std::vector<std::reference_wrapper<Motor>> motors) : motors(motors), motorCount(motors.size()) {};
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
        /// @brief Stops all motors with the default stop mode
        inline void stop() {
            for (auto &motor : motors) {
                motor.get().stop();
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

// controller.h
namespace vpp {
    class Controller {
    private:
        vex::controller controller;

    public:
        Controller() {};
        // Axis Position
        inline float leftY() { return controller.Axis3.position(); };
        inline float leftX() { return controller.Axis4.position(); };
        inline float rightX() { return controller.Axis1.position(); };
        inline float rightY() { return controller.Axis2.position(); };
        // Buttons
        inline bool ButtonUp() { return controller.ButtonUp.pressing(); };
        inline bool ButtonDown() { return controller.ButtonDown.pressing(); };
        inline bool ButtonLeft() { return controller.ButtonLeft.pressing(); };
        inline bool ButtonRight() { return controller.ButtonRight.pressing(); };
        inline bool ButtonA() { return controller.ButtonA.pressing(); };
        inline bool ButtonB() { return controller.ButtonB.pressing(); };
        inline bool ButtonX() { return controller.ButtonX.pressing(); };
        inline bool ButtonY() { return controller.ButtonY.pressing(); };
        inline bool ButtonL1() { return controller.ButtonL1.pressing(); };
        inline bool ButtonL2() { return controller.ButtonL2.pressing(); };
        inline bool ButtonR1() { return controller.ButtonR1.pressing(); };
        inline bool ButtonR2() { return controller.ButtonR2.pressing(); };
        inline void vibrate(const char *str) { controller.rumble(str); };
        inline void vibrate(const std::string str) { controller.rumble(str.c_str()); };
        // Screen
        // TODO: Experiment with options & design API around it
    };
};  // namespace vpp

// drivetrain.h
#define CLAMP(expr, lower, upper) std::max(lower, std::min(expr, upper))
namespace vpp {
    class Drivetrain {
    public:
        MotorGroup &leftGroup, &rightGroup;
        Drivetrain(MotorGroup &left, MotorGroup &right) : leftGroup(left), rightGroup(right) {};
        /// @brief Drives with arcade controls
        /// @param lateral Lateral input
        /// @param angular Angular input
        /// @ref https://wiki.purduesigbots.com/software/robotics-basics/arcade-drive
        inline void arcade(float lateral, float angular) {
            leftGroup.spin(lateral + angular);
            rightGroup.spin(lateral - angular);
        };
        inline void arcade(float lateral, float angular, float maxSpeed) {
            tank(CLAMP(lateral + angular, -maxSpeed, maxSpeed), CLAMP(lateral - angular, -maxSpeed, maxSpeed));
        }
        /// @brief Drives with tank controls
        /// @param left Left motor group speed
        /// @param right Right motor group speed
        inline void tank(float left, float right) {
            leftGroup.spin(left);
            rightGroup.spin(right);
        };
        /// @brief Sets the default brake mode for both motor groups
        /// @param mode COAST, BRAKE, or HOLD
        inline void setStopMode(MotorStopMode mode) {
            leftGroup.setStopMode(mode);
            rightGroup.setStopMode(mode);
        };
    };
};  // namespace vpp
