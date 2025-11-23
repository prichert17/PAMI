#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <array>
#include "motor.hpp"
#include "../utils/coordinates.hpp"

class Odometry {
private:
    std::array<Wheel, 3>* wheels;
    Vector2DAndRotation position;
    Vector2DAndRotation velocity;
    
    class SerialOut* serial;
    
    float wheel_base;  // Distance entre les roues
    
public:
    Odometry(std::array<Wheel, 3>& wheels);
    
    void attach_serial(SerialOut* serial_ptr);
    void update_odometry();
    void print_position();
    
    Vector2DAndRotation get_position() const { return position; }
    Vector2DAndRotation get_velocity() const { return velocity; }
    Vector2DAndRotation get_speed_filtered() const { return velocity; }  // Alias pour compatibilité
    
    void reset_position(float x = 0, float y = 0, float theta = 0);
};

#endif