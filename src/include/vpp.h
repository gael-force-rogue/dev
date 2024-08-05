#pragma once

#include "vpp/controller.h"
#include "vpp/drivetrain.h"
#include "vpp/motor.h"

#include "vex.h"

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