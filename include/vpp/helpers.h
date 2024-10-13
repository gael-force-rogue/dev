#pragma once

#include "vex.h"

#define CLAMP(expr, threshold) std::max(-threshold, std::min(expr, threshold))

namespace vpp {
    /// @brief Sleeps the current thread for a specified amount of time
    /// @param time Time to wait in milliseconds
    inline void sleep(int time) { vex::this_thread::sleep_for(time); };
}