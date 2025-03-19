#include "./devices/controller.h"
#include "./algorithms/pid.h"

using namespace vpp;

class PIDTuner {
public:
    Controller controller;

    PIDTuner(Controller &controller, PIDConstants initialConstants) {
        this->controller = controller;
    };
};