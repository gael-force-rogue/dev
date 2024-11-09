#ifndef TUNER_H
#define TUNER_H

#include <map>
#include <tuple>
#include <vector>

#include "pid.h"
#include "vex.h"

enum ConfigTunerAction {
    START,
    STOP,
    MODIFIED,
    UNCHANGED
};

class ConfigTuner {
private:
    controller Controller;

public:
    float lastIndex;
    int currentIndex = 0;
    std::vector<std::string> keys;
    std::vector<float> values;

    float incrementBy;

    /// @brief Construct a new Config Tuner object
    /// @param Controller vex::controller
    /// @param incrementBy How much to increment or decrement a value by
    /// @param keys Names of tunable values (Must be same length as "values")
    /// @param values Initial values of tunable values (Must be same length as "keys")
    ConfigTuner(controller &Controller, float incrementBy, std::vector<std::string> keys, std::vector<float> values);

    void render();
    ConfigTunerAction checkForAction();

    // Choose what constant is tunable now
    void previous();
    void next();

    // Increment or decrement the current constant
    void increment();
    void decrement();
};

#endif  // TUNER_H