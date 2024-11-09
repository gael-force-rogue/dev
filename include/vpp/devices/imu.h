#pragma once

#include "vex.h"
#include "vpp/helpers.h"

using namespace vpp;

namespace vpp {
    class IMU : private vex::inertial {
    public:
        IMU(int port) : vex::inertial(port) {};

        /**
         * @brief Calibrates the IMU for certain period of time - blocking
         */
        void calibrate() {
            this->calibrate();
            while (this->isCalibrating()) {
                wait(2, msec);
            }
        };

        /**
         * @brief Returns the current heading of the robot
         * @return Heading in degrees normalized to -180 to 180
         */
        inline float heading() {
            return normalize180(this->heading());
        };

        inline void reset() {
            this->resetHeading();
            this->resetRotation();
        };
    };
}  // namespace vpp