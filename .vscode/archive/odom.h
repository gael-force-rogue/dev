#ifndef ODOM_H
#define ODOM_H

enum OdometrySourceType {
    TRACKER,
    ENCODER
};

using OdometryState = float[3];

class OdometryController {
  private:
    float left_offset, right_offset, sideways_offset, wheel_radius;
    float left_position = 0, right_position = 0, sideways_position = 0;

  public:
    OdometryState state = {0, 0, 0};

    OdometryController(float left_tracker_offset, float right_tracker_offset, float sideways_tracker_offset, float wheel_radius);

    void update(float left_position_degrees, float right_position_degrees, float sideways_position_degrees);
};

#endif // ODOM_H