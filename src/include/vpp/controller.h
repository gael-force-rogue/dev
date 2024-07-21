#pragma once

#include "vex.h"
#include <string>

namespace vpp {
    class Controller {
    private:
        vex::controller vex_controller;

    public:
        Controller(){};

        // Axis Position
        inline float leftY() { return vex_controller.Axis3.position(); };
        inline float leftX() { return vex_controller.Axis4.position(); };
        inline float rightX() { return vex_controller.Axis1.position(); };
        inline float rightY() { return vex_controller.Axis2.position(); };

        // Buttons
        inline bool buttonUp() { return vex_controller.ButtonUp.pressing(); };
        inline bool buttonDown() { return vex_controller.ButtonDown.pressing(); };
        inline bool buttonLeft() { return vex_controller.ButtonLeft.pressing(); };
        inline bool buttonRight() { return vex_controller.ButtonRight.pressing(); };

        inline bool buttonA() { return vex_controller.ButtonA.pressing(); };
        inline bool buttonB() { return vex_controller.ButtonB.pressing(); };
        inline bool buttonX() { return vex_controller.ButtonX.pressing(); };
        inline bool buttonY() { return vex_controller.ButtonY.pressing(); };

        inline bool buttonL1() { return vex_controller.ButtonL1.pressing(); };
        inline bool buttonL2() { return vex_controller.ButtonL2.pressing(); };
        inline bool buttonR1() { return vex_controller.ButtonR1.pressing(); };
        inline bool buttonR2() { return vex_controller.ButtonR2.pressing(); };

        inline void vibrate(const char *str) { vex_controller.rumble(str); };
        inline void vibrate(const std::string str) { vex_controller.rumble(str.c_str()); };

        // Screen
        // TODO: Experiment with options & design API around it
    };
};  // namespace vpp