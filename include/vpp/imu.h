#pragma once

#include "vex.h"

namespace vpp {
    class IMU {
    private:
        vex::inertial inertial;

    public:
        IMU();

        float roll();
        float pitch();
        float yaw();
    };
}  // namespace vpp
