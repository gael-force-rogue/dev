#pragma once

#include "vex.h"
#include <string>

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
        inline bool Up() { return controller.ButtonUp.pressing(); };
        inline bool Down() { return controller.ButtonDown.pressing(); };
        inline bool Left() { return controller.ButtonLeft.pressing(); };
        inline bool Right() { return controller.ButtonRight.pressing(); };

        inline bool A() { return controller.ButtonA.pressing(); };
        inline bool B() { return controller.ButtonB.pressing(); };
        inline bool X() { return controller.ButtonX.pressing(); };
        inline bool Y() { return controller.ButtonY.pressing(); };

        inline bool L1() { return controller.ButtonL1.pressing(); };
        inline bool L2() { return controller.ButtonL2.pressing(); };
        inline bool R1() { return controller.ButtonR1.pressing(); };
        inline bool R2() { return controller.ButtonR2.pressing(); };

        inline void vibrate(const char *str) { controller.rumble(str); };
        inline void vibrate(const std::string str) { controller.rumble(str.c_str()); };

        // Screen
        // TODO: Experiment with options & design API around it
    };
};  // namespace vpp