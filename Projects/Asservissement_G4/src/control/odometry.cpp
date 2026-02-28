#include "odometry.hpp"
#include "utils/printf.hpp"
#include <cmath>

Odometry::Odometry(std::array<Wheel, 3>& wheels)
    : wheels(&wheels), serial(nullptr), wheel_base(CONSTANTS::WHEEL_BASE),
      position(0, 0, 0), velocity(0, 0, 0)
{
}

void Odometry::attach_serial(SerialOut* serial_ptr) {
    serial = serial_ptr;
}

void Odometry::update_odometry() {
    static const float dt = 0.002f;  // 500Hz = 2ms
    
    // Mise à jour des vitesses des roues
    for (auto& wheel : *wheels) {
        wheel.update_speed(dt);
    }
    
    // Pour 2 roues différentielles (roues 0 et 1)
    float v_left = (*wheels)[0].get_speed() * CONSTANTS::WHEEL_RADIUS;
    float v_right = (*wheels)[1].get_speed() * CONSTANTS::WHEEL_RADIUS;
    
    // Calcul de la vitesse linéaire et angulaire du robot
    float v_linear = (v_right + v_left) / 2.0f;
    float omega = (v_right - v_left) / wheel_base;
    
    // Mise à jour de la position
    position.teta += omega * dt;
    
    // Normalisation de l'angle entre -PI et PI
    while (position.teta > M_PI) position.teta -= 2.0f * M_PI;
    while (position.teta < -M_PI) position.teta += 2.0f * M_PI;
    
    // Mise à jour des coordonnées X et Y
    position.x_y.x += v_linear * cos(position.teta) * dt;
    position.x_y.y += v_linear * sin(position.teta) * dt;
    
    // Mise à jour des vitesses
    velocity.x_y.x = v_linear * cos(position.teta);
    velocity.x_y.y = v_linear * sin(position.teta);
    velocity.teta = omega;
}

void Odometry::print_position() {
    if (serial) {
        serial->printf("Pos: X=%.2f Y=%.2f Theta=%.2f\n", 
                      position.x_y.x,
                      position.x_y.y,
                      position.teta * 180.0f / M_PI);
    }
}

void Odometry::reset_position(float x, float y, float theta) {
    position.x_y.x = x;
    position.x_y.y = y;
    position.teta = theta;
}
