#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include <cmath>
#include "tim.h"
#include "../utils/constants.hpp"

class Wheel {
private:
    uint8_t wheel_id;
    float encoder_step_per_rev;
    float wheel_radius;
    float angle_offset;  // Angle en degrés (0° pour gauche, 180° pour droite)
    
    TIM_HandleTypeDef* encoder_timer;
    TIM_HandleTypeDef* pwm_timer;
    uint32_t pwm_channel;
    uint32_t pwm_channel_n;
    
    int32_t last_encoder_count;
    float current_speed;

public:
    Wheel(uint8_t id, float encoder_steps, float radius, float angle);
    
    void init_timers();
    void set_pwm(int16_t duty_cycle);  // -1000 à +1000
    void set_motor_power(int32_t power) { set_pwm((int16_t)power); }  // Alias pour compatibilité
    int32_t get_encoder_count();
    float get_speed();
    void update_speed(float dt);
    
    float get_angle_rad() const { return angle_offset * M_PI / 180.0f; }
    float get_wheel_weight_x() const { return cos(get_angle_rad()); }
    float get_wheel_weight_y() const { return sin(get_angle_rad()); }
};

#endif