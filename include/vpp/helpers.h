#pragma once

#include "vex.h"

#define CLAMP(expr, threshold) std::max(-threshold, std::min(expr, threshold))

#define IF_BUTTON_PRESS(expression, action, sleep) \
    if (expression) {                              \
        action;                                    \
        while (expression) {                       \
            vex::this_thread::sleep_for(10);       \
        }                                          \
    }

#define ELSE_IF_BUTTON_PRESS(expression, action, sleep) \
    else BUTTON_PRESS(expression, action, sleep)

namespace vpp {
    /// @brief Sleeps the current thread for a specified amount of time
    /// @param time Time to wait in milliseconds
    inline void sleep(int time) { vex::this_thread::sleep_for(time); };

    /// @brief Normalizes an angle to be between -180 and 180
    /// @param angle Angle to normalize (degrees)
    inline float normalize180(float angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    };

    /// @brief Normalizes an angle to be between -360 and 360
    /// @param angle Angle to normalize (degrees)
    inline float normalize360(float angle) {
        while (angle >= 360) angle -= 360;
        while (angle <= -360) angle += 360;
        return angle;
    };

    const int PORTS[20] = {vex::PORT1, vex::PORT2, vex::PORT3, vex::PORT4, vex::PORT5, vex::PORT6, vex::PORT7, vex::PORT8, vex::PORT9, vex::PORT10, vex::PORT11, vex::PORT12, vex::PORT13, vex::PORT14, vex::PORT15, vex::PORT16, vex::PORT17, vex::PORT18, vex::PORT19, vex::PORT20};
    // const float resolvePort(unsigned int port) {
    //     if (port < 1 || port > 20) {
    //         exit(1);
    //     }

    //     return PORTS[port - 1];
    // };
}  // namespace vpp