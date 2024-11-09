#pragma once

#include "vpp.h"

enum DeviceControlType {
    DRIVER,
    ROBOT
};

enum AllianceColor {
    RED,
    BLUE
};

class Intake : public Motor {
private:
    DeviceControlType control = DRIVER;

    vex::optical *colorSensor;
    AllianceColor allianceColor;
    float ringDetectionThreshold;

    /**
     * @brief Checks color sensor for enemy ring
     */
    inline bool enemyRingDetected() {
        auto rgb = colorSensor->getRgb();
        return (allianceColor == RED ? rgb.blue : rgb.red) > ringDetectionThreshold;
    };

public:
    bool colorSortingEnabled = false;

    Intake(int port, bool reverse) : Motor(port, reverse) {};

    inline Intake &withColorSorting(vex::optical &colorSensor, AllianceColor allianceColor, float ringDetectionThreshold = 100) {
        this->colorSensor = &colorSensor;
        this->colorSortingEnabled = true;
        this->allianceColor = allianceColor;
        this->ringDetectionThreshold = ringDetectionThreshold;
        return *this;
    };

    inline void forward(float velocity = 100) {
        this->spin(velocity);
    };

    inline void reverse(float velocity = 100) {
        this->spin(-velocity);
    };

    /**
     * @brief Handles driver control of intake
     * @param forwardPressed Whether the forward button is pressed
     * @param backwardPressed Whether the backward button is pressed
     */
    inline void handleDrivercontrol(bool forwardPressed, bool backwardPressed) {
        if (forwardPressed) {
            control = DRIVER;
            forward();
        } else if (backwardPressed) {
            control = DRIVER;
            reverse();
        } else if (control == DRIVER) {
            stop();
        }
    };

    void searchForEnemyRings();
};

class Lift : public Motor {
private:
    float defaultPosition;
    Intake &intake;
    DeviceControlType control = DRIVER;

public:
    Lift(int port, bool reverse, float defaultPosition, Intake &intake) : Motor(port, reverse), defaultPosition(defaultPosition), intake(intake) {};

    inline float relativePosition() {
        return normalize360(position());
    };

    inline void spinToRelativePosition(float angle, bool waitForCompletion) {
        spinToPosition(position() - relativePosition() + angle, waitForCompletion);
    };

    /**
     * @brief Handles driver control of lift
     * @param forwardPressed Whether the forward button is pressed
     * @param backwardPressed Whether the backward button is pressed
     */
    inline void handleDrivercontrol(bool forwardPressed, bool backwardPressed) {
        if (forwardPressed) {
            control = DRIVER;
            this->spin(100);
        } else if (backwardPressed) {
            control = DRIVER;
            this->spin(-100);
        } else if (control == DRIVER) {
            if (relativePosition() > defaultPosition && relativePosition() < 180) {
                returnToDefaultPosition(false);
            } else {
                stop();
            }
        }
    };

    /**
     * @brief Returns lift to default position
     * @param waitForCompletion Whether to wait for the lift to reach the default position
     */
    inline void returnToDefaultPosition(bool waitForCompletion) {
        spinToRelativePosition(defaultPosition, true);
    };

    void score();
};