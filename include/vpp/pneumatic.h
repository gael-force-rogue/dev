#include "vex.h"
#include "vpp.h"

namespace vpp {
    class Pneumatic {
    private:
        vex::pneumatics pneumatic;
        bool isDeployed = false;

    public:
        Pneumatic(vex::triport::port port) : pneumatic(vex::pneumatics(port)) {};

        inline void toggle() {
            if (isDeployed) {
                close();
            } else {
                open();
            }
            isDeployed = !isDeployed;
        }

        inline float value() {
            return pneumatic.value();
        };

        inline void open() {
            pneumatic.open();
        };

        inline void close() {
            pneumatic.close();
        }
    };
}  // namespace vpp
