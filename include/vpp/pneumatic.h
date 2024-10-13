#include "vex.h"
#include "vpp.h"

namespace vpp {
    class Pneumatic {
    private:
        vex::pneumatics pneumatic;

    public:
        Pneumatic(vex::triport::port port) : pneumatic(vex::pneumatics(port)) {};

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
