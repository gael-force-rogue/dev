#pragma once

#include "vex.h"
#include <cmath>

#define CLAMP(expr, threshold) std::max(-threshold, std::min(expr, threshold))
#define CLAMP_SPECIFIC(expr, mi, ma) std::max(mi, std::min(expr, ma))

#define IF_BUTTON_PRESS(expression, action, sleep) \
    if (expression) {                              \
        action;                                    \
        while (expression) {                       \
            vex::this_thread::sleep_for(10);       \
        }                                          \
    }

#define ELSE_IF_BUTTON_PRESS(expression, action, sleep) \
    else BUTTON_PRESS(expression, action, sleep)

#define DEADZONE(value, threshold) (fabs(value) > threshold ? value : 0)

namespace vpp {
    /// @brief Sleeps the current thread for a specified amount of time
    /// @param time Time to wait in milliseconds
    inline void sleep(int time) { vex::this_thread::sleep_for(time); };

    /// @brief Normalizes an angle to be between -90 and 90
    /// @param angle Angle to normalize (degrees)
    inline float normalize90(float angle) {
        while (angle > 90) angle -= 180;
        while (angle < -90) angle += 180;
        return angle;
    };

    /// @brief Normalizes an angle to be between -180 and 180
    /// @param angle Angle to normalize (degrees)
    inline float normalize180(float angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    };

    /// @brief Normalizes an angle to be between -360 and 360
    /// @param angle Angle to normalize (degrees)
    inline float normalize360(float angle) {
        while (angle >= 360) angle -= 360;
        while (angle <= -360) angle += 360;
        return angle;
    };

    /**
     * @brief Calculates the conversion factor from inches to degrees for a given wheel
     * @param wheelDiameter Diameter of the wheel (inches)
     * @param externalRatio External gear ratio of the wheel (input:output)
     */
    const inline float calculateInchesToDegreesConversionFactor(float wheelDiameter, float externalRatio) {
        return (360 * externalRatio) / (wheelDiameter * M_PI);
    };

    /**
     * @brief Calculates the conversion factor from degrees to inches for a given wheel
     * @param wheelDiameter Diameter of the wheel (inches)
     * @param externalRatio External gear ratio of the wheel (input:output)
     */
    const inline float calculateDegreesToInchesConversionFactor(float wheelDiameter, float externalRatio) {
        return (wheelDiameter * M_PI) / (360 * externalRatio);
    };
}  // namespace vpp