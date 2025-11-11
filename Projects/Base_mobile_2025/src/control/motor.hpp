#ifndef __MOTOR_NEW__
#define __MOTOR_NEW__

#include <stdint.h>
#include <algorithm>
#include <math.h>
#include "utils/unit.hpp"
#include "ST_files/gpio.h"
#include "ST_files/tim.h"
#include "utils/constants.hpp"
#include "utils/printf.hpp"
#include "utils/filter.hpp"

/*
    This class is used to manage the encoder of a motor
*/
class Encoder{
    private:
        const uint8_t encoder_num;  // Number of the encoder
        SerialOut *serial;          // Pointer to the serial output

        /**
         * @brief Read the value of the encoder
         * @return Value of the encoder
         */
        int32_t get_encoder_value();    // Read the value of the encoder

    protected:
        const uint16_t encoder_step_per_rev;    // Number of step per revolution of the encoder
        volatile int32_t nb_step_encoder_now;   // Number of step of the encoder now
        volatile int32_t nb_step_encoder_last;  // Number of step of the encoder at the last period

    public:
        /**
         * @brief Construct a new Encoder object
         * @param encoder_num Physical number of the encoder
         * @param encoder_step_per_rev Number of step per revolution of the encoder
         *  */ 
        Encoder(uint8_t encoder_num, uint16_t encoder_step_per_rev);

        /**
         * @brief Get the stored number of step of the encoder now
         * @return Stored number of step of the encoder now
         */
        int32_t get_nb_step_now();

        /**
         * @brief Update the stored value of the encoder
         */
        void update_encoder();          
};

/*
    This class is used to manage a motor
*/
class Motor{
    private: 
        const uint8_t motor_num;    // Number of the motor
        int32_t pwm_value;          // Value of the PWM currently set
        SerialOut *serial;          // Pointer to the serial output

    public:
        /**
         * @brief Construct a new Motor object
         * @param motor_num Physical number of the motor
         *  */ 
        Motor(uint8_t motor_num);

        /**
         * @brief Set the power of the motor
         * @param pwm_value Power of the motor between -(2^16)-1 and (2^16)-1, the sign controlls the direction
         *  */
        void set_motor_power(int32_t pwm_value);    // Set the power of the motor
};

/*
    This class is used to manage a wheel, which is a motor with an encoder attached to a wheel
*/
class Wheel : public Encoder, public Motor {
    private:
        const uint8_t wheel_num;    // Number of the wheel
        const mm wheel_radius;      // Radius of the wheel
        const float weight_x;       // Weight of the wheel on the x axis, used for odometry calculation
        const float weight_y;       // Weight of the wheel on the y axis, used for odometry calculation
        SerialOut *serial;          // Pointer to the serial output

    protected:
        Filter_Generic<rad_per_s> speed_filter;        // Rolling average filter to smooth the calculated speed of the wheel
        rad_per_s rads_wheel_now;   // Speed of the wheel in rad/s
        mm_per_s rads_wheel_filtered;   // Filtered speed of the wheel in rad/s
    
    public:
        /**
         * @brief Construct a new Wheel object
         * @param wheel_num Physical number of the wheel
         * @param encoder_step_per_rev Number of step per revolution of the encoder
         * @param wheel_radius Radius of the wheel
         * @param wheel_angle Angle of the wheel in the robot referential
         *  */    
        Wheel(uint8_t wheel_num, uint16_t encoder_step_per_rev, mm wheel_radius, deg wheel_angle);

        /**
         * @brief Update the speed of the wheel
         *  */
        void update_wheel_speed();

        /**
         * @brief Get the speed of the wheel
         * @return Speed of the wheel in rad/s
         *  */
        rad_per_s get_wheel_rads_now();

        /**
         * @brief Get the speed of the wheel
         * @return Speed of the wheel in mm/s
         *  */
        mm_per_s get_wheel_mms_now();

        /**
         * @brief Get the filtered speed of the wheel
         * @return Filtered speed of the wheel in rad/s
         *  */
        rad_per_s get_wheel_rads_filtered();

        /**
         * @brief Get the filtered speed of the wheel
         * @return Filtered speed of the wheel in mm/s
         *  */
        mm_per_s get_wheel_mms_filtered();

        /**
         * @brief Get the weight of the wheel on the x axis
         * @return Weight of the wheel on the x axis
         *  */
        float get_wheel_weight_x();

        /**
         * @brief Get the weight of the wheel on the y axis
         * @return Weight of the wheel on the y axis
         *  */
        float get_wheel_weight_y();
    };


#endif