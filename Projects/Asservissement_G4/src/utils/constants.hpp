#ifndef CONSTANTS
#define CONSTANTS

#include "unit.hpp"

namespace CONSTANTS{

    inline constexpr uint8_t WHEEL_NUMBER = 3;

    inline constexpr mm WHEEL_RADIUS = 29.0;

    inline constexpr mm BASE_RADIUS = 150.0;
    
    inline constexpr mm WHEEL_BASE = 150.0;  // Distance entre les 2 roues motrices

    inline constexpr uint16_t ENCODER_STEP_REV = 1200; 

    inline constexpr double ODOMETRY_FREQ = 500.0;

    inline constexpr uint32_t PRINTF_BUFFER_SIZE = 512;

    inline constexpr uint8_t FILTER_SIZE = 50;

    inline constexpr double ODOMETRY_CORRECTION_XY = 2.07;

    inline constexpr double ASSERV_FREQ = 100.0;

    inline constexpr double ROTATION_DYNAMIC_RANGE = 200.0;
}

#endif