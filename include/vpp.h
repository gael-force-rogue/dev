#pragma once

#include "vpp/chassis.h"
#include "vpp/controller.h"
#include "vpp/motor.h"
#include "vpp/odom.h"

#include "vex.h"

#define BUTTON_PRESS(expression, action, sleep, modifier) \
    modifier if (expression) {                            \
        action;                                           \
        while (expression) {                              \
            vex::this_thread::sleep_for(10);              \
        }                                                 \
    }

#define BUTTON_PRESS(expression, action, sleep) \
    if (expression) {                           \
        action;                                 \
        while (expression) {                    \
            vex::this_thread::sleep_for(10);    \
        }                                       \
    }
