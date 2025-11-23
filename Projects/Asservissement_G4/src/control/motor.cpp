#include "motor.hpp"

Wheel::Wheel(uint8_t id, float encoder_steps, float radius, float angle)
    : wheel_id(id), encoder_step_per_rev(encoder_steps), 
      wheel_radius(radius), angle_offset(angle),
      last_encoder_count(0), current_speed(0.0f)
{
    // Attribution des timers selon l'ID de la roue
    if (id == 1) {
        encoder_timer = &htim2;
        pwm_timer = &htim1;
        pwm_channel = TIM_CHANNEL_1;
        pwm_channel_n = TIM_CHANNEL_1;  // Pour PWMN
    } else if (id == 2) {
        encoder_timer = &htim3;
        pwm_timer = &htim16;
        pwm_channel = TIM_CHANNEL_1;
        pwm_channel_n = TIM_CHANNEL_1;  // Pour PWMN
    } else if (id == 3) {
        encoder_timer = nullptr;  // Pas d'encodeur pour la roue 3
        pwm_timer = &htim8;
        pwm_channel = TIM_CHANNEL_1;
        pwm_channel_n = TIM_CHANNEL_1;
    }
}

void Wheel::set_pwm(int16_t duty_cycle) {
    // Limitation à ±1000
    if (duty_cycle > 1000) duty_cycle = 1000;
    if (duty_cycle < -1000) duty_cycle = -1000;
    
    uint16_t pwm_value = abs(duty_cycle);
    
    if (duty_cycle >= 0) {
        // Sens positif
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pwm_value);
        // Canal complémentaire à 0
        if (pwm_timer->Instance == TIM1) {
            TIM1->CCR1 = pwm_value;
        } else if (pwm_timer->Instance == TIM16) {
            TIM16->CCR1 = pwm_value;
        }
    } else {
        // Sens négatif - utiliser le canal complémentaire
        // Inverser la logique selon votre câblage
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, 0);
    }
}

int32_t Wheel::get_encoder_count() {
    if (encoder_timer == nullptr) return 0;
    return (int32_t)(__HAL_TIM_GET_COUNTER(encoder_timer)) - (1 << 15);
}

void Wheel::update_speed(float dt) {
    int32_t current_count = get_encoder_count();
    int32_t delta = current_count - last_encoder_count;
    last_encoder_count = current_count;
    
    // Vitesse en rad/s
    current_speed = (delta / encoder_step_per_rev) * (2.0f * M_PI) / dt;
}

float Wheel::get_speed() {
    return current_speed;
}