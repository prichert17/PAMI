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

void Asserv::set_motors_power_relative(Vector2DAndRotation power){
    for(auto wheel : *wheels){
        double speed = (2.0/3.0)*wheel.get_wheel_weight_x()*power.x_y.x + (2.0/3.0)*wheel.get_wheel_weight_y()*power.x_y.y + (1.0/3.0)*power.teta;
        wheel.set_motor_power((int32_t)speed); 
    }
}

void Asserv::set_motors_power_absolute(Vector2DAndRotation power){
    Vector2DAndRotation power_loc = power.rotate_only_vector(-odometry->get_position().teta);
    for(auto wheel : *wheels){
        double speed = (2.0/3.0)*wheel.get_wheel_weight_x()*power_loc.x_y.x + (2.0/3.0)*wheel.get_wheel_weight_y()*power_loc.x_y.y + (1.0/3.0)*power_loc.teta;
        wheel.set_motor_power((int32_t)speed); 
    }
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
    asserv_started(false),
    mode_differentiel(false)
    {

    }

void Asserv_Position::set_mode_differentiel(bool enable) {
    mode_differentiel = enable;
}

void Asserv_Position::set_motors_power_absolute_differentiel(Vector2DAndRotation power){
    // Rotation du vecteur de commande dans le repère local du robot
    Vector2DAndRotation power_loc = power.rotate_only_vector(-odometry->get_position().teta);
    
    // Extraction des consignes
    double consigne_vx = power_loc.x_y.x;           // Vitesse linéaire (avant/arrière)
    double consigne_v_theta = power_loc.teta;       // Vitesse angulaire (rotation)
    
    // Cinématique inverse différentielle
    // Moteur 1 = Droit (wheels[0])
    double target_speed_motor1 = consigne_vx + (consigne_v_theta * CONSTANTS::ENTRE_AXE / 2.0);
    
    // Moteur 2 = Gauche (wheels[1])
    double target_speed_motor2 = consigne_vx - (consigne_v_theta * CONSTANTS::ENTRE_AXE / 2.0);
    
    // Moteur 3 = Forcé à 0 (wheels[2]) - Conservé mais inactif
    double target_speed_motor3 = 0.0;
    
    // Appliquer les consignes aux moteurs
    (*wheels)[0].set_motor_power((int32_t)target_speed_motor1);  // Moteur Droit
    (*wheels)[1].set_motor_power((int32_t)target_speed_motor2);  // Moteur Gauche
    (*wheels)[2].set_motor_power((int32_t)target_speed_motor3);  // Moteur 3 à 0
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
    position_error_last = position_error_now;
    position_error_now = target_position - odometry->get_position();
    position_error_now.teta *= CONSTANTS::ROTATION_DYNAMIC_RANGE; 
    error_P = position_error_now;
    error_I += position_error_now/CONSTANTS::ASSERV_FREQ;
    error_D = (position_error_now - position_error_last)*CONSTANTS::ASSERV_FREQ;
    command = error_P*P + error_I*I + error_D*D;
    command.teta /= CONSTANTS::ROTATION_DYNAMIC_RANGE;
    command_limiter(1<<16);
    
    // Choisir la cinématique selon le mode
    if (mode_differentiel) {
        set_motors_power_absolute_differentiel(command);
    } else {
        set_motors_power_absolute(command);  // Mode holonome (ancien code)
    }
}

void Asserv_Position::stop_asserv(){

}

void Asserv_Position::set_target_position(Vector2DAndRotation target_position){
    this->target_position = target_position;
}

void Asserv_Position::set_motors_power_relative(Vector2DAndRotation power){
    for(auto wheel : *wheels){
        double speed = (2.0/3.0)*wheel.get_wheel_weight_x()*power.x_y.x + (2.0/3.0)*wheel.get_wheel_weight_y()*power.x_y.y + (1.0/3.0)*power.teta;
        wheel.set_motor_power((int32_t)speed); 
    }
}

void Asserv_Position::set_motors_power_absolute(Vector2DAndRotation power){
    Vector2DAndRotation power_loc = power.rotate_only_vector(-odometry->get_position().teta);
    for(auto wheel : *wheels){
        double speed = (2.0/3.0)*wheel.get_wheel_weight_x()*power_loc.x_y.x + (2.0/3.0)*wheel.get_wheel_weight_y()*power_loc.x_y.y + (1.0/3.0)*power_loc.teta;
        wheel.set_motor_power((int32_t)speed); 
    }
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