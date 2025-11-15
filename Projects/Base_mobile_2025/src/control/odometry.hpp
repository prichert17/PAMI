#ifndef __ODOMETRY_NEW__
#define __ODOMETRY_NEW__

#include <stdint.h>
#include <array>
#include <algorithm>
#include "utils/unit.hpp"
#include "ST_files/gpio.h"
#include "ST_files/tim.h"
#include "utils/constants.hpp"
#include "utils/coordinates.hpp"
#include "motor.hpp"

/*
    This class is used to manage the odometry of the robot
*/
class Odometry{
    private:
        std::array<Wheel, 3> wheels;    // Array of the wheels of the robot
        SerialOut *serial;              // Pointer to the serial output
        
        // Variables pour odométrie différentielle
        int32_t last_ticks_droit;
        int32_t last_ticks_gauche;
        bool mode_differentiel;  // Flag pour basculer entre holonome et différentiel
        
    protected:
        Vector2DAndRotation position_now;   // Position of the robot now
        Vector2DAndRotation position_last;  // Position of the robot at the last period
        Vector2DAndRotation speed_now;      // Speed of the robot now
        Vector2DAndRotation speed_filtered; // Filtered speed of the robot
        Filter_Vector2DAndRotation speed_filter;    // Rolling average filter to smooth the calculated speed of the robot

    public:
        /**
         * @brief Construct a new Odometry object
         * @param wheels Array of the wheels of the robot
         *  */
        Odometry(std::array<Wheel, 3> wheels);

        /**
         * @brief Update the odometry of the robot - Must be called at a fixed frequency in an interrupt
         */
        void update_odometry();

        /**
         * @brief Update the odometry for differential drive
         */
        void update_odometry_differentielle();

        /**
         * @brief Set the differential drive mode
         * @param enable true to enable differential mode, false for holonomic mode
         */
        void set_mode_differentiel(bool enable);

        /**
         * @brief Print the position of the robot
         */
        void print_position();

        /**
         * @brief Print the speed of the robot
         */
        void print_speed();

        /**
         * @brief Print the filtered speed of the robot
         */
        void print_speed_filtered();

        /**
         * @brief Attach a serial output to the odometry
         * @param _serial Pointer to a serial output
         */
        void attach_serial(SerialOut *_serial);

        /**
         * @brief Get the position of the robot
         * @return Position of the robot
         */
        Vector2DAndRotation get_position();

        /**
         * @brief Get the speed of the robot now
         * @return Speed of the robot
         */
        Vector2DAndRotation get_speed_now();

        /**
         * @brief Get the filtered speed of the robot
         * @return Filtered speed of the robot
         */
        Vector2DAndRotation get_speed_filtered();
};


#endif