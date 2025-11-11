#include "odometry.hpp"


Odometry::Odometry(std::array<Wheel, 3> wheels):
    wheels(wheels),
    serial(NULL),
    position_now(Vector2DAndRotation(0.0, 0.0, 0.0)),
    position_last(Vector2DAndRotation(0.0, 0.0, 0.0)),
    speed_now(Vector2DAndRotation(0.0, 0.0, 0.0)),
    speed_filtered(Vector2DAndRotation(0.0, 0.0, 0.0))
{
    HAL_TIM_Base_Start_IT(&htim6);
}

void Odometry::update_odometry(){
    // CPU time : 1.79%

    Vector2DAndRotation local_displacement; // Displacement of the robot in the local frame of reference
    rad teta = 0.0; // Angular displacement of the robot in the local frame of reference since the last period
    mm x = 0.0;     // Linear x displacement of the robot in the local frame of reference since the last period
    mm y = 0.0;     // Linear y displacement of the robot in the local frame of reference since the last period

    // Iterate over all the wheels of the robot:
    for(auto &wheel : wheels){
        wheel.update_encoder(); // Update the encoder of the wheel
        wheel.update_wheel_speed(); // Update the speed of the wheel
        const mm_per_s wheel_speed = wheel.get_wheel_mms_now();
        // Compute the contribution of the wheel to the displacement of the robot
        teta += wheel_speed;
        x += wheel_speed*wheel.get_wheel_weight_x();
        y += wheel_speed*wheel.get_wheel_weight_y();
    }

    // Scale the contributions and integrate them over the period to get the displacement of the robot
    teta *= 1/(3*CONSTANTS::BASE_RADIUS*CONSTANTS::ODOMETRY_FREQ);
    x /= 3.0*CONSTANTS::ODOMETRY_FREQ;
    y /= 3.0*CONSTANTS::ODOMETRY_FREQ;

    // Buid a Vector2DAndRotation object with the displacement of the robot
    local_displacement.teta = teta;
    local_displacement.x_y = Vector2D(x, y);
    // Cursed correction to get correct values...
    local_displacement.x_y.x *= CONSTANTS::ODOMETRY_CORRECTION_XY;
    local_displacement.x_y.y *= CONSTANTS::ODOMETRY_CORRECTION_XY;
    // Update the position of the robot in the global frame of reference
    position_last = position_now;
    position_now += local_displacement.rotate_only_vector(position_now.teta + local_displacement.teta);

    // Compute the speed of the robot in the global frame of reference
    speed_now = (position_now-position_last)*CONSTANTS::ODOMETRY_FREQ;
    // Filter the speed of the robot
    speed_filter.add_elem(speed_now);
    speed_filtered = speed_filter.get_average();
}

void Odometry::print_position(){
    if(serial == NULL) return;
    serial->printf("Position:\tx: ");
    serial->printf_decimal(position_now.x_y.x, 9);
    serial->printf("\ty: ");
    serial->printf_decimal(position_now.x_y.y, 9);
    serial->printf("\tteta: ");
    serial->printf_decimal(position_now.teta, 9);
    serial->printf("\n");
}

void Odometry::print_speed(){
    if(serial == NULL) return;
    serial->printf("Speed:\tx: ");
    serial->printf_decimal(speed_now.x_y.x, 9);
    serial->printf("\ty: ");
    serial->printf_decimal(speed_now.x_y.y, 9);
    serial->printf("\tteta: ");
    serial->printf_decimal(speed_now.teta, 9);
    serial->printf("\n");
}

void Odometry::print_speed_filtered(){
    if(serial == NULL) return;
    serial->printf("Speed:\tx: ");
    serial->printf_decimal(speed_filtered.x_y.x, 9);
    serial->printf("\ty: ");
    serial->printf_decimal(speed_filtered.x_y.y, 9);
    serial->printf("\tteta: ");
    serial->printf_decimal(speed_filtered.teta, 9);
    serial->printf("\n");
}

void Odometry::attach_serial(SerialOut *_serial){
    this->serial = _serial;
}

Vector2DAndRotation Odometry::get_position(){
    return position_now;
}

Vector2DAndRotation Odometry::get_speed_now(){
    return speed_now;
}

Vector2DAndRotation Odometry::get_speed_filtered(){
    return speed_filtered;
}
