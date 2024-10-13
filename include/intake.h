#pragma once

#include "vpp/motor.h"
#include "vex.h"

#include <iostream>

// TODO: Implement intake states
enum IntakeState {
    ALL_IN,
    LAUNCH_FAR_OPPONENTS,
    LAUNCH_FAR_NEXT,
    LAUNCH_NEAR_OPPONENTS,
    LAUNCH_NEAR_NEXT,
};

class Intake {
private:
    vpp::Motor intake;
    vex::optical opticalSensor;

public:
    Intake(vpp::Motor motor, vex::optical opticalSensor) : intake(motor), opticalSensor(opticalSensor) {};

    void in() {
        intake.spin(100);
    };

    void out() {
        intake.spin(-100);
    };

    void stop() {
        intake.stop();
    };

    bool isRunning() {
        return intake.velocity() != 0;
    }

    void handleRing() {
        if (opticalSensor.isNearObject()) {
            auto rgb = opticalSensor.getRgb();
            std::cout << rgb.red << " " << rgb.green << " " << rgb.blue << " " << rgb.clear << "\n";
        }
    };
};