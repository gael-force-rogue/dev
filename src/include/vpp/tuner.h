#pragma once

#include "vpp/controller.h"
#include <map>
#include <string>
#include <vector>

namespace vpp {
    enum ConfigTunerAction {
        RUN,
        STOP,
        CHANGED,
        NOTHING
    };

    class ConfigTuner {
    private:
        Controller &controller;
        std::vector<std::string> keys;
        std::vector<float> values;

        int currentIndex = 0, maxIndex;
        float incrementBy, deadzone;

    public:
        /// @brief hi
        /// @param controller
        /// @param keys
        /// @param values
        /// @param incrementBy
        /// @param deadzone
        ConfigTuner(Controller &controller, std::vector<std::string> keys, std::vector<float> values,
                    float incrementBy = 0.1, float deadzone = 20);

        void render();
        ConfigTunerAction checkForAction();
    };
}  // namespace vpp