#pragma once

#include "vex.h"

namespace vpp {
    class Pneumatic {
    private:
        vex::pneumatics pneumatic;
        bool isDeployed = false;

    public:
        /**
         * @brief Creates a new Pneumatic object
         * @param port Port of the Pneumatic
         * @param isDeployed Default state of the Pneumatic - make sure this matches your pneumatic wiring
         */
        Pneumatic(vex::triport::port port) : pneumatic(vex::pneumatics(port)) {};

        inline void toggle() {
            if (isDeployed) {
                close();
            } else {
                open();
            }
            isDeployed = !isDeployed;
        }

        inline void open() {
            pneumatic.open();
        };

        inline void close() {
            pneumatic.close();
        }
    };
}  // namespace vpp
