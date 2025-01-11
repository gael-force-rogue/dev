// Define custom devices such as Intake & Lift

#pragma once

#include "vpp.h"
#include "autons.h"

using namespace vpp;

enum LiftState {
    STANDBY = 0,
    LOAD = 20,
    SCORE = 140
};

class Lift {
private:
    Motor motor;

public:
    LiftState state;
    bool macroRunning = false;

    Lift(int port, bool reverse) : motor(port, GREEN_18, reverse) {};

    inline void handleDrivercontrol(bool up, bool down) {
        if (up) {
            state = SCORE;
            macroRunning = false;
            motor.spin(100);
        } else if (down) {
            state = SCORE;
            macroRunning = false;
            motor.spin(-100);
        } else if (!macroRunning) {
            motor.stop();
        };
    };

    /**
     * @brief Toggles between the LiftState.
     */
    inline void toggleState() {
        macroRunning = true;

        if (state == STANDBY) {
            state = LOAD;
        } else if (state == LOAD) {
            state = SCORE;
        } else {
            state = STANDBY;
        };

        motor.spinToPosition(state, true);

        macroRunning = false;
    };
};

class Intake {
private:
    Motor motor;
    vex::optical optical;
    Lift &lift;

    bool launchingEnemyRing = false;

public:
    vex::color opponentColor = IS_ALLIANCE_RED ? vex::color::blue : vex::color::red;

    Intake(int port, bool reverse, vex::optical optical, Lift &lift)
        : motor(port, GREEN_18, reverse), optical(optical), lift(lift) {};

    /**
     * @brief Turn on the optical sensor
     */
    inline void enable() {
        optical.setLightPower(100);
        optical.gestureDisable();
    };

    /**
     * @brief Turn off the optical sensor
     */
    inline void disable() {
        optical.setLightPower(0);
    };

    /**
     * @brief Handle's drivecontrol
     * @param inb In button
     * @param outb Out button
     */
    inline void handleDrivercontrol(bool inb, bool outb) {
        if (!launchingEnemyRing) {
            if (inb) {
                motor.spin(100);
            } else if (outb) {
                motor.spin(-100);
            } else {
                motor.stop();
            }
        };
    };

    /**
     * @brief Handle's the color sort macro
     */
    inline void handleColorSortMacro() {
        if (optical.color() == opponentColor && motor.velocity() != 0) {
            launchingEnemyRing = true;

            motor.stop(HOLD);
            sleep(300);

            launchingEnemyRing = false;
        };
    };
};