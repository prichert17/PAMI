#ifndef __ASSERVISSEMENT_NEW__
#define __ASSERVISSEMENT_NEW__

#include <stdint.h>
#include <array>
#include <algorithm>
#include "utils/unit.hpp"
#include "utils/constants.hpp"
#include "utils/coordinates.hpp"
#include "motor.hpp"
#include "odometry.hpp"
#include "utils/printf.hpp"



class Asserv{
    private:
        Odometry *odometry;
        std::array<Wheel, 3> *wheels;
        Vector2DAndRotation target_speed;
        Vector2DAndRotation speed_error_now;
        Vector2DAndRotation speed_error_last;
        Vector2DAndRotation error_P;
        Vector2DAndRotation error_I;
        Vector2DAndRotation error_D;
        Vector2DAndRotation command;
        double P;
        double I;
        double D;
        bool asserv_started;

    public:
        Asserv(Odometry *odometry, std::array<Wheel, 3> *wheels);

        void set_PID(double P, double I, double D);

        void start_asserv();
        void update_asserv();
        void stop_asserv();
        void set_target_speed(Vector2DAndRotation target_speed);

        void set_motors_power_relative(Vector2DAndRotation power);
        void set_motors_power_absolute(Vector2DAndRotation power);
};

class Asserv_Position{
    private:
        SerialOut *serial;
        Odometry *odometry;
        std::array<Wheel, 3> *wheels;
        Vector2DAndRotation target_position;
        Vector2DAndRotation position_error_now;
        Vector2DAndRotation position_error_last;
        Vector2DAndRotation error_P;
        Vector2DAndRotation error_I;
        Vector2DAndRotation error_D;
        Vector2DAndRotation command;
        double P;
        double I;
        double D;
        bool asserv_started;

    public:
        Asserv_Position(Odometry *odometry, std::array<Wheel, 3> *wheels);

        void set_PID(double P, double I, double D);

        void start_asserv();
        void update_asserv();
        void stop_asserv();
        void set_target_position(Vector2DAndRotation target_position);

        void set_motors_power_relative(Vector2DAndRotation power);
        void set_motors_power_absolute(Vector2DAndRotation power);
        void command_limiter(double max_power);
        void attach_serial(SerialOut *_serial);
        void print_command();
};

#endif