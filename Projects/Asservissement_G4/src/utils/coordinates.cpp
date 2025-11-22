#include "coordinates.hpp"

rad frame_angle(rad angle) {
    while (angle < M_PI)
        angle += M_PI*2;
    while (angle > M_PI)
        angle -= M_PI*2;
    return angle;
}

float set_abs_float(float value, float desired_abs) {
    if (value < 0)
        return -abs(desired_abs);
    if (value > 0)
        return abs(desired_abs);
    return 0;
}
