#pragma once

#include "vpp/controller.h"
#include "vpp/helpers.h"
#include <map>
#include <string>
#include <sstream>
#include <vector>

namespace vpp {
    std::string floatToString(float n) {
        std::stringstream ss;
        ss << n;
        return ss.str();
    };

    enum TunerAction {
        START,
        NOTHING
    };

    class Tuner {
    private:
        Controller &controller;
        int currentKeyIndex = 0;

    public:
        std::vector<std::string> keys;
        std::vector<float> values;

        Tuner(Controller &controller, std::vector<std::string> keys, std::vector<float> values)
            : controller(controller), keys(keys), values(values) {
            if ((keys.size() != values.size()) || (keys.size() == 0)) {
                throw std::invalid_argument("Keys and values must be the same size & greater than 0!");
            }
        };

        TunerAction update() {
            BUTTON_PRESS(controller.ButtonUp(), values[currentKeyIndex]++, 20);
            BUTTON_PRESS(controller.ButtonDown(), values[currentKeyIndex]--, 20);
            BUTTON_PRESS(controller.ButtonLeft(), currentKeyIndex--, 20);
            BUTTON_PRESS(controller.ButtonRight(), currentKeyIndex++, 20);
            if (controller.ButtonA()) {
                return START;
            }

            controller.screen.clearScreen();
            controller.screen.setCursor(1, 1);
            // controller.screen.print(floatToString(keys[currentKeyIndex]));
            controller.screen.setCursor(2, 1);
            // controller.screen.print(floatToString(values[currentKeyIndex]));

            return NOTHING;
        };
    };
}  // namespace vpp