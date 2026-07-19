#include "motor.hpp"

Encoder::Encoder(uint8_t encoder_num, uint16_t encoder_step_per_rev):
    encoder_num(encoder_num),
    serial(NULL),
    encoder_step_per_rev(encoder_step_per_rev),
    nb_step_encoder_now(0),
    nb_step_encoder_last(0)
    {
    switch(encoder_num){
        // Sets the base value of the encoder and starts it
        case 1:                    
            HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
            TIM1->CNT = (1<<15);
            break;
        case 2:
            HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
            TIM2->CNT = (1<<15);
            break;
        case 3:
            HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
            TIM3->CNT = (1<<15);
            break;
        default:
            break;
    }   
}

int32_t Encoder::get_encoder_value(){
    // Each encoder uses a timer with a 16 bits counter. The counter is set to 2^15 to avoid undeflows if the counter is decremented.
    // 2^15 if considered to be the 0 value of the encoder.
    // 2^15 is substracted to the counter value to get the real value of the encoder, and the counter is reset to 2^15.
    
    int32_t value = 0;
    switch(encoder_num){
        case 1:
            value = TIM1->CNT - (1<<15);
            TIM1->CNT = (1<<15);
            return value;
        case 2:
            value = TIM2->CNT - (1<<15);
            TIM2->CNT = (1<<15);
            return value;
        case 3:
            value = TIM3->CNT - (1<<15);
            TIM3->CNT = (1<<15);
            return value;
        default:
            return 0;
    }
}

int32_t Encoder::get_nb_step_now(){
    return nb_step_encoder_now;
}

void Encoder::update_encoder(){
    nb_step_encoder_last = nb_step_encoder_now;
    nb_step_encoder_now += get_encoder_value();     
}




Motor::Motor(uint8_t motor_num):
    motor_num(motor_num),
    pwm_value(0),
    serial(NULL)
    {
    // Starts the corresponding PWM timer
    switch(motor_num){
        case 1:
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
            break;
        case 2:
            HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
            break;
        case 3:
            HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
            break;
    }
}

void Motor::set_motor_power(int32_t pwm_value){
    // Each motor is controlled by a PWM signal. The PWM value is between 0 an (2^16)-1.
    // The direction of the motor is controlled by the sign of the value passed as argument.
    // A GPIO pin is set to 0 or 1 to control the direction of the motor.
    // The value passed as argument is clamped to (2^16)-1 to avoid overflows.
    if(pwm_value >= 0){
        const uint16_t speed = pwm_value >= (1<<16) ? (1<<16)-1 : pwm_value; // Clamp the speed to 2^16-1
        switch(motor_num){
            case 1:
                TIM4->CCR2 = speed;
                HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
                break;
            case 2:
                TIM8->CCR1 = speed;
                HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
                break;
            case 3:
                TIM17->CCR1 = speed;
                HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_RESET);
                break;
        }
    }
    else{
        const uint16_t speed = -pwm_value >= (1<<16) ? (1<<16)-1 : pwm_value; // Clamp the speed to 2^16-1
        switch(motor_num){
            case 1:
                TIM4->CCR2 = speed;
                HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
                break;
            case 2:
                TIM8->CCR1 = speed;
                HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
                break;
            case 3:
                TIM17->CCR1 = speed;
                HAL_GPIO_WritePin(DIR_3_GPIO_Port, DIR_3_Pin, GPIO_PIN_SET);
                break;
        }
    }
}

 
 Wheel::Wheel(uint8_t wheel_num, uint16_t encoder_step_per_rev, mm wheel_radius, deg wheel_angle):
    Encoder(wheel_num, encoder_step_per_rev),
    Motor(wheel_num),
    wheel_num(wheel_num),
    wheel_radius(wheel_radius),
    weight_x(cos(M_PI_2 + wheel_angle*M_PI/180.0)),
    weight_y(sin(M_PI_2 + wheel_angle*M_PI/180.0)),
    serial(NULL),
    rads_wheel_now(0.0),
    rads_wheel_filtered(0.0){}

void Wheel::update_wheel_speed(){
    rads_wheel_now = CONSTANTS::ODOMETRY_FREQ*M_PI*2*(nb_step_encoder_now-nb_step_encoder_last)/(float)encoder_step_per_rev;    // Instantaneous speed of the wheel in rad/s
    speed_filter.add_elem(rads_wheel_now);  // Filter the speed
    rads_wheel_filtered = speed_filter.get_average();
}

rad_per_s Wheel::get_wheel_rads_now(){
    return rads_wheel_now;
}

mm_per_s Wheel::get_wheel_mms_now(){
    return rads_wheel_now*wheel_radius;
}

rad_per_s Wheel::get_wheel_rads_filtered(){
    return rads_wheel_filtered;
}

mm_per_s Wheel::get_wheel_mms_filtered(){
    return rads_wheel_filtered*wheel_radius;
}

float Wheel::get_wheel_weight_x(){
    return weight_x;
}

float Wheel::get_wheel_weight_y(){
    return weight_y;
}
   
