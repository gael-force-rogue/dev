#include "vpp/helpers.h"

namespace vpp {
    struct Dataset {
    };

    class Brain {
    private:
        vex::brain brain;

    public:
        Brain();
        ~Brain();

        void send(std::string data) {
            printf("Sensor Value: %d\n", sensorValue);
            delay(1000);
        };
    };
}  // namespace vpp