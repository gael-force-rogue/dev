#pragma once

#include "vex.h"
#include "vpp/helpers.h"

using namespace vpp;

namespace vpp {
    class IMU {
    private:
        vex::inertial inertial;

    public:
        IMU(int port) : inertial(port) {};

        /**
         * @brief Calibrates the IMU for certain period of time - blocking
         */
        void calibrate() {
            inertial.startCalibration();
            while (inertial.isCalibrating()) {
                sleep(2);
            };
        };

        /**
         * @brief Checks if the IMU is connected
         * @return True if the IMU is connected
         */
        inline bool connected() {
            return inertial.installed();
        };

        /**
         * @brief Returns the current heading of the robot
         * @return Heading in degrees normalized to -180 to 180
         */
        inline float heading() {
            return normalize180(inertial.heading());
        };

        inline void reset() {
            inertial.resetHeading();
            inertial.resetRotation();
        };
    };
}  // namespace vpp