#include "asservissement.hpp"

Asserv::Asserv(Odometry *odometry, std::array<Wheel, 3> *wheels):
    odometry(odometry),
    wheels(wheels),
    target_speed(Vector2DAndRotation(0,0,0)),
    speed_error_now(Vector2DAndRotation(0,0,0)),
    speed_error_last(Vector2DAndRotation(0,0,0)),
    error_P(Vector2DAndRotation(0,0,0)),
    error_I(Vector2DAndRotation(0,0,0)),
    error_D(Vector2DAndRotation(0,0,0)),
    P(0),
    I(0),
    D(0),
    asserv_started(false)
    {

    }

void Asserv::set_PID(double P, double I, double D){
    this->P = P;
    this->I = I;
    this->D = D;
}

void Asserv::start_asserv(){
    asserv_started = true;
}

void Asserv::update_asserv(){
    if(!asserv_started) return;
    speed_error_last = speed_error_now;
    speed_error_now = target_speed - odometry->get_speed_filtered();
    error_P = speed_error_now;
    error_I += speed_error_now/CONSTANTS::ASSERV_FREQ;
    error_D = (speed_error_now - speed_error_last)*CONSTANTS::ASSERV_FREQ;
    command = error_P*P + error_I*I + error_D*D;
    set_motors_power_absolute(command);
}

void Asserv::stop_asserv(){

}

void Asserv::set_target_speed(Vector2DAndRotation target_speed){
    this->target_speed = target_speed;
}

// Commande directe en Linéaire (X, Y) et Angulaire (Z)
void Asserv::set_motors_power_relative(Vector2DAndRotation power){
    double cmd_linear = power.x_y.x;  // Avance/Recule
    double cmd_rot    = power.teta;   // Tourne

    // Roue 0 = Gauche (M1), Roue 1 = Droite (M2)
    
    // Roue Gauche = V - W
    double speed_left  = cmd_linear - cmd_rot;
    // Roue Droite = V + W
    double speed_right = cmd_linear + cmd_rot;

    (*wheels)[0].set_motor_power((int32_t)speed_left);
    (*wheels)[1].set_motor_power((int32_t)speed_right);
}

void Asserv::set_motors_power_absolute(Vector2DAndRotation power){
// Rotation du vecteur commande dans le référentiel du robot
    Vector2DAndRotation power_loc = power.rotate_only_vector(-odometry->get_position().teta);
    set_motors_power_relative(power_loc);
}


Asserv_Position::Asserv_Position(Odometry *odometry, std::array<Wheel, 3> *wheels):
    serial(NULL),
    odometry(odometry),
    wheels(wheels),
    target_position(Vector2DAndRotation(0,0,0)),
    position_error_now(Vector2DAndRotation(0,0,0)),
    position_error_last(Vector2DAndRotation(0,0,0)),
    error_P(Vector2DAndRotation(0,0,0)),
    error_I(Vector2DAndRotation(0,0,0)),
    error_D(Vector2DAndRotation(0,0,0)),
    P(0),
    I(0),
    D(0),
    asserv_started(false)
    {

    }

void Asserv_Position::set_PID(double P, double I, double D){
    this->P = P;
    this->I = I;
    this->D = D;
}

void Asserv_Position::start_asserv(){
    asserv_started = true;
}

void Asserv_Position::update_asserv(){
    if(!asserv_started) return;

    Vector2DAndRotation current_pos = odometry->get_position();

    double dx = target_position.x_y.x - current_pos.x_y.x;
    double dy = target_position.x_y.y - current_pos.x_y.y;
    double dist_to_target = sqrt(dx*dx + dy*dy);

    // Arrivé à destination -> stop
    if (dist_to_target < 20.0) {
        command = Vector2DAndRotation(0, 0, 0);
        set_motors_power_relative(command);
        return;
    }

    // Calcul angle vers cible
    double angle_to_target = atan2(dy, dx);
    double angle_error = angle_to_target - current_pos.teta;
    while (angle_error > M_PI)  angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // PID sur l'angle
    position_error_last = position_error_now;
    position_error_now.teta = angle_error;
    position_error_now.x_y.x = dist_to_target;

    error_P = position_error_now;
    error_I += position_error_now / CONSTANTS::ASSERV_FREQ;
    error_D = (position_error_now - position_error_last) * CONSTANTS::ASSERV_FREQ;

    // Anti-windup
    if (error_I.teta > 50.0) error_I.teta = 50.0;
    if (error_I.teta < -50.0) error_I.teta = -50.0;

    // Hystérésis: seuils différents pour entrer/sortir du mode rotation
    static bool in_rotation_mode = true;
    double threshold_enter = 0.4;  // ~23° pour passer en rotation
    double threshold_exit = 0.15;  // ~9° pour sortir de rotation

    if (fabs(angle_error) > threshold_enter) in_rotation_mode = true;
    else if (fabs(angle_error) < threshold_exit) in_rotation_mode = false;

    // Phase 1: Mode rotation
    if (in_rotation_mode) {
        double cmd_rot = error_P.teta * P + error_I.teta * I + error_D.teta * D;
        // Boost initial pour vaincre l'inertie
        double min_cmd = 80.0;
        if (fabs(cmd_rot) < min_cmd && fabs(angle_error) > 0.05)
            cmd_rot = (cmd_rot > 0) ? min_cmd : -min_cmd;
        command.teta = cmd_rot;
        command.x_y.x = 0;
    } 
    // Phase 2: Bien orienté -> avancer
    else {
        command.teta = error_P.teta * P * 0.5;
        command.x_y.x = error_P.x_y.x * P;
    }
    command.x_y.y = 0;

    command_limiter(400.0);
    set_motors_power_relative(command);
}

void Asserv_Position::stop_asserv(){
    asserv_started = false;
    // Reset de l'intégrateur pour éviter un saut au redémarrage
    error_I = Vector2DAndRotation(0, 0, 0);
}

void Asserv_Position::set_target_position(Vector2DAndRotation target_position){
    this->target_position = target_position;
}

void Asserv_Position::set_motors_power_relative(Vector2DAndRotation power){
    double cmd_linear = power.x_y.x;
    double cmd_rot    = power.teta;

    // Roue 0 = Gauche, Roue 1 = Droite
    // Pour tourner à droite (teta négatif), gauche doit aller plus vite
    // Pour tourner à gauche (teta positif), droite doit aller plus vite
    double speed_left  = cmd_linear + cmd_rot;  // Inversé: + au lieu de -
    double speed_right = cmd_linear - cmd_rot;  // Inversé: - au lieu de +

    (*wheels)[0].set_motor_power((int32_t)speed_left);
    (*wheels)[1].set_motor_power((int32_t)speed_right);
}

void Asserv_Position::set_motors_power_absolute(Vector2DAndRotation power){
    set_motors_power_relative(power);
}

void Asserv_Position::command_limiter(double max_power){
    double max_componant = abs(command.teta);
    if(abs(command.x_y.x) > max_componant) max_componant = abs(command.x_y.x);
    if(abs(command.x_y.y) > max_componant) max_componant = abs(command.x_y.y);
    if(max_componant > max_power) command = command*max_power/max_componant;    
}

void Asserv_Position::attach_serial(SerialOut *_serial){
    this->serial = _serial;
}

void Asserv_Position::print_command(){
    if(serial == NULL) return;
    serial->printf("Commande:\tx: ");
    serial->printf_decimal(command.x_y.x, 9);
    serial->printf("\ty: ");
    serial->printf_decimal(command.x_y.y, 9);
    serial->printf("\tteta: ");
    serial->printf_decimal(command.teta, 9);
}