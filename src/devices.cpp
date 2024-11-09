#include "devices.h"

void Intake::searchForEnemyRings() {
    if (!colorSortingEnabled) return;

    colorSensor->setLightPower(100);

    while (true) {
        if (enemyRingDetected()) {
            control = ROBOT;
            forward();
            wait(200, msec);
            stop(HOLD);
            control = DRIVER;
        };

        sleep(20);
    };
};

void Lift::score() {
    spinToRelativePosition(100, true);
    intake.forward();
    spinToRelativePosition(200, true);
    returnToDefaultPosition(true);
    intake.stop();
};