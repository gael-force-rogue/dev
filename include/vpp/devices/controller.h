#pragma once

#include "vex.h"
#include <string>

// mult line while loop bocker
#define CONTROLLER_BINDING(controller, binding) \
    while (controller.Button##binding()) {      \
        vex::this_thread::sleep_for(20);        \
    }

namespace vpp {
    class Controller {
    private:
        vex::controller controller;

    public:
        vex::controller::lcd screen;

        Controller() : controller(vex::controllerType::primary) {
            this->screen = controller.Screen;
        };

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

        // Rumble
        inline void vibrate(const char *str) { controller.rumble(str); };
        inline void vibrate(const std::string str) { controller.rumble(str.c_str()); };

        // Screen
        // TODO: Experiment with options & design API around it
    };
};  // namespace vpp