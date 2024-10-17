#pragma once

#include "vex.h"

#define CLAMP(expr, threshold) std::max(-threshold, std::min(expr, threshold))

#define BUTTON_PRESS(expression, action, sleep) \
    if (expression) {                           \
        action;                                 \
        while (expression) {                    \
            vex::this_thread::sleep_for(10);    \
        }                                       \
    }

#define BUTTON_PRESS_ELSE(expression, action, sleep) \
    else BUTTON_PRESS(expression, action, sleep)

namespace vpp {
    /// @brief Sleeps the current thread for a specified amount of time
    /// @param time Time to wait in milliseconds
    inline void sleep(int time) { vex::this_thread::sleep_for(time); };
}  // namespace vpp