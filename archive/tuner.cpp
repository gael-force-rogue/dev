#include <tuner.h>

#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "pid.h"
#include "vex.h"

// Generic Configuration Tuner
ConfigTuner::ConfigTuner(controller &Controller, float incrementBy, std::vector<std::string> keys, std::vector<float> values)
    : Controller(Controller), keys(keys), values(values), incrementBy(incrementBy) {
    if (keys.size() != values.size()) {
        std::cerr << "Keys and values must be the same size" << std::endl;
        exit(1);
    }

    lastIndex = keys.size() - 1;
    this->keys = keys;
    this->values = values;
};

ConfigTunerAction ConfigTuner::checkForAction() {
    // if statements for each arrow key
    if (Controller.ButtonUp.pressing()) {
        increment();
    } else if (Controller.ButtonDown.pressing()) {
        decrement();
    } else if (Controller.ButtonLeft.pressing()) {
        previous();
    } else if (Controller.ButtonRight.pressing()) {
        next();
    } else if (Controller.ButtonA.pressing()) {
        return ConfigTunerAction::START;
    } else if (Controller.ButtonB.pressing()) {
        return ConfigTunerAction::STOP;
    } else {
        return ConfigTunerAction::UNCHANGED;
    }

    // The the last 3 if statements didn't return, that means a value was modified
    return ConfigTunerAction::MODIFIED;
};

void ConfigTuner::render() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(0, 1);
    Controller.Screen.print(keys[currentIndex].c_str());
    Controller.Screen.newLine();
    Controller.Screen.print(values[currentIndex]);
};

// Data Manipulation
void ConfigTuner::previous() {
    if (currentIndex == 0) {
        currentIndex = lastIndex;
    } else {
        currentIndex--;
    }
};

void ConfigTuner::next() {
    if (currentIndex == lastIndex) {
        currentIndex = 0;
    } else {
        currentIndex++;
    }
};

void ConfigTuner::increment() {
    values[currentIndex] += incrementBy;
};

void ConfigTuner::decrement() {
    values[currentIndex] -= incrementBy;
};