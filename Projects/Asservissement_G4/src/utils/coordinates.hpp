#ifndef COORDINATES
#define COORDINATES

#include <cmath>

#include "unit.hpp"
#include "constants.hpp"

class Vector2D {
   public:
    float x, y;

    Vector2D() {
        this->x = 0;
        this->y = 0;
    }
    Vector2D(float x, float y) {
        this->x = x;
        this->y = y;
    }

    Vector2D operator+(Vector2D const another_coord) const {
        return Vector2D(this->x + another_coord.x, this->y + another_coord.y);
    }
    Vector2D operator-(Vector2D const another_coord) const {
        return Vector2D(this->x - another_coord.x, this->y - another_coord.y);
    }
    Vector2D operator*(const float lambda) const {
        return Vector2D(this->x * lambda, this->y * lambda);
    }
    Vector2D operator/(const float lambda) const {
        return Vector2D(this->x / lambda, this->y / lambda);
    }
    Vector2D& operator+=(const Vector2D& other) {
        this->x += other.x;
        this->y += other.y;
        return *this;
    }

    float angle() const {
        if (this->y != 0 || this->x != 0)
            return atan2(this->y, this->x);
        else
            return 0;
    }

    Vector2D rotate(rad angle) const {
        float new_angle = this->angle() + angle;
        float abs = this->get_abs();
        return Vector2D(cos(new_angle) * abs, sin(new_angle) * abs);
    }

    float get_abs() const { return sqrt(pow(this->x, 2) + pow(this->y, 2)); }
    
    Vector2D set_abs(float abs) {
        const float this_abs = this->get_abs();
        if (this_abs == 0)
            return *this;
        else
            return (*this) * (abs / this_abs);
    }

    Vector2D cap_abs(float abs) {
        const float this_abs = this->get_abs();
        if (this_abs == 0 || this_abs <= abs)
            return *this;
        else
            return (*this) * (abs / this_abs);
    }

    Vector2D project_on(Vector2D target_vector) const {
        float target_angle = target_vector.angle();
        float new_abs = this->get_abs() * cos(target_angle - this->angle());
        return Vector2D(cos(target_angle) * new_abs,
                        sin(target_angle) * new_abs);
    }

    float scalar_product(Vector2D other_vector) const {
        return this->x * other_vector.x + this->y * other_vector.y;
    }

    static Vector2D unit_vector(rad angle) {
        return Vector2D(cos(angle), sin(angle));
    }

    void print(char str[]) const {
        // [DISABLED]
    }
};

class Vector2DAndRotation {
   public:
    Vector2D x_y;
    float teta;

    Vector2DAndRotation() {
        // this->x_y default construct to (0, 0)
        this->teta = 0;
    }
    Vector2DAndRotation(float x, float y, float teta) {
        this->x_y = Vector2D(x, y);
        this->teta = teta;
    }
    Vector2DAndRotation(Vector2D x_y, float teta) {
        this->x_y = x_y;
        this->teta = teta;
    }

    Vector2DAndRotation operator+(
        const Vector2DAndRotation another_coord) const {
        return Vector2DAndRotation(this->x_y + another_coord.x_y,
                                   this->teta + another_coord.teta);
    }
    Vector2DAndRotation operator-(
        const Vector2DAndRotation another_coord) const {
        return Vector2DAndRotation(this->x_y - another_coord.x_y,
                                   this->teta - another_coord.teta);
    }
    Vector2DAndRotation operator*(const float lambda) const {
        return Vector2DAndRotation(this->x_y * lambda, this->teta * lambda);
    }
    Vector2DAndRotation operator/(const float lambda) const {
        return Vector2DAndRotation(this->x_y / lambda, this->teta / lambda);
    }
    Vector2DAndRotation& operator+=(const Vector2DAndRotation& other) {
        this->x_y += other.x_y;
        this->teta += other.teta;
        return *this;
    }

    float get_abs() const { return this->x_y.get_abs(); }

    Vector2DAndRotation rotate_from_origin(rad angle) const {
        return Vector2DAndRotation(this->x_y.rotate(angle), this->teta + angle);
    }
    Vector2DAndRotation rotate_locally(rad angle) const {
        return Vector2DAndRotation(this->x_y, this->teta + angle);
    }
    Vector2DAndRotation rotate_only_vector(rad angle) const {
        return Vector2DAndRotation(this->x_y.rotate(angle), this->teta);
    }

    void print() const {
        // [DISABLED]
    }
    void print(char str[]) const {
        // [DISABLED]
    }
};

typedef Vector2DAndRotation Position;      // mm
typedef Vector2DAndRotation Distance;      // mm
typedef Vector2DAndRotation Speed;         // mm / second
typedef Vector2DAndRotation Acceleration;  // mm / second / second

rad frame_angle(rad angle);
float set_abs_float(float value, float desired_abs);

#endif
