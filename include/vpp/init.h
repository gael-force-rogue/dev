#include "vex.h"
#include "vpp/helpers.h"

#include "vpp/devices/controller.h"

// Since these are universal and VPP may need it, we define it for you
extern vex::competition Compeittion;
extern Controller mainController;
extern vex::brain Brain;

#define BRAIN_PRINTF(line, format, ...) \
    Brain.Screen.setCursor(line, 1);    \
    Brain.Screen.clearLine();           \
    Brain.Screen.print(format, __VA_ARGS__);

#define BRAIN_PRINTLN(line, text)    \
    Brain.Screen.setCursor(line, 1); \
    Brain.Screen.clearLine();        \
    Brain.Screen.print(text);