#pragma once

#include "vex.h"

namespace vpp {
    class Rotation {
    private:
        vex::rotation rotation = vex::rotation(0);

    public:
        Rotation(int port, bool reversed) {
            rotation = vex::rotation(port, reversed);
        };
    };
}  // namespace vpp