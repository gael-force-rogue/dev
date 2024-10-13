#pragma once

#include "vpp/controller.h"
#include <map>
#include <string>
#include <vector>

namespace vpp {
    class Tuner {
        std::vector<std::string> keys;
        std::vector<float> values;

    private:
    public:
        Tuner(std::vector<std::string> keys, std::vector<float> values) {
            if (keys.size() != values.size()) {
                throw std::invalid_argument("Keys and values must be the same size");
            }

            this->keys = keys;
            this->values = values;
        };

        void update() {

        };
    };
}  // namespace vpp