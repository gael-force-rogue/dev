#include "odom.h"
#include <cmath>
#include <complex>

// M_PI / 180 -> 3.14159265358979323846 / 180 -> 0.01745329251
#define DEGREES_TO_RADIANS 0.01745329251;

// Function to convert cartesian coordinates to polar coordinates
std::pair<float, float> cartesianToPolar(float x, float y) {
    float r = std::sqrt(x * x + y * y);
    float theta = std::atan2(y, x);
    return std::make_pair(r, theta);
}

float degreesToRadians(float degrees) {
    return degrees * DEGREES_TO_RADIANS;
}

float chordLength(float radius, float angle_radians) {
    return 2 * radius * sin(angle_radians / 2);
}

using OdometryState = float[3];

class OdometryController {
  private:
    float left_offset, right_offset, sideways_offset;
    float left_position = 0, right_position = 0, sideways_position = 0;
    OdometryState state = {0, 0, 0};

  public:
    OdometryController(float left_tracker_offset, float right_tracker_offset, float sideways_tracker_offset) {
        this->left_offset = left_tracker_offset;
        this->right_offset = right_tracker_offset;
        this->sideways_offset = sideways_tracker_offset;
    };

    void update(float left_position_degrees, float right_position_degrees, float sideways_position_degrees) {
        // Convert to radians
        float left_position = degreesToRadians(left_position_degrees);
        float right_position = degreesToRadians(right_position_degrees);
        float sideways_position = degreesToRadians(sideways_position_degrees);

        // Caluculate change in position
        float delta_left_position = left_position - this->left_position;
        float delta_right_position = right_position - this->right_position;
        float delta_sideways_position = sideways_position - this->sideways_position;
        this->left_position = left_position;
        this->right_position = right_position;
        this->sideways_position = sideways_position;

        // Step 5
        float orientation = (delta_left_position - delta_right_position) / (this->left_offset + this->right_offset);
        // Step 6
        float delta_theta = orientation - this->state[2];

        float local_x, local_y;
        if (delta_theta == 0) {
            local_x = delta_sideways_position;
            local_y = delta_right_position;
        } else {
            local_x = chordLength(delta_theta, delta_sideways_position / delta_theta + this->sideways_offset);
            local_y = delta_right_position / delta_theta + this->right_offset;
        }

        float average_orientation = this->state[2] + (delta_theta / 2);

        float local_polar_angle = 0, local_polar_length = 0;
        if (local_x != 0 && local_y != 0) {
            local_polar_angle = atan2(local_y, local_x);
            // Sovle for c in Pythagorean theorem
            local_polar_length = sqrt(local_x * local_x + local_y * local_y);
        }

        // If this is addition, then the global x and y deltas have to be rotated by -(global_polar_angle)
        // I think we (and JARTmplate) can avoid that by just subtracting here
        float global_polar_angle = local_polar_angle - average_orientation;

        float global_x = local_polar_length * cos(global_polar_angle);
        float global_y = local_polar_length * sin(global_polar_angle);

        this->state[0] += global_x;
        this->state[1] += global_y;
        this->state[2] += delta_theta;
    };
};
