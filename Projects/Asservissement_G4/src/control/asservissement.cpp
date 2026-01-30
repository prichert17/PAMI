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

    // 1. Calcul de l'erreur vectorielle
    double dx = target_position.x_y.x - current_pos.x_y.x;
    double dy = target_position.x_y.y - current_pos.x_y.y;
    
    // 2. Conversion en Distance (Rho) et Angle (Alpha)
    double dist_to_target = sqrt(dx*dx + dy*dy);
    double angle_to_target = atan2(dy, dx);
    double angle_error = angle_to_target - current_pos.teta;

    // Normalisation angle (-PI à PI)
    while (angle_error > M_PI)  angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // 3. Gestion de la marche arrière (Si la cible est derrière, on recule)
    if (angle_error > M_PI_2 || angle_error < -M_PI_2) {
        dist_to_target = -dist_to_target;
        angle_error = angle_error > 0 ? angle_error - M_PI : angle_error + M_PI;
    }

    // 4. Si on est très proche, on s'aligne sur l'angle final (Target Theta)
    if (abs(dist_to_target) < 10.0) { // 10mm de tolérance
        double final_angle_err = target_position.teta - current_pos.teta;
        while (final_angle_err > M_PI)  final_angle_err -= 2.0 * M_PI;
        while (final_angle_err < -M_PI) final_angle_err += 2.0 * M_PI;
        angle_error = final_angle_err;
    }

    // 5. On "triche" en mettant ces valeurs polaires dans le vecteur erreur
    // .x = Erreur Linéaire
    // .teta = Erreur Angulaire
    // .y = Inutilisé
    position_error_last = position_error_now;
    position_error_now.x_y.x = dist_to_target;
    position_error_now.x_y.y = 0; 
    position_error_now.teta  = angle_error;

    // 6. PID Classique
    error_P = position_error_now;
    error_I += position_error_now/CONSTANTS::ASSERV_FREQ;
    error_D = (position_error_now - position_error_last)*CONSTANTS::ASSERV_FREQ;

    // On applique des gains. 
    // ATTENTION: P, I, D sont globaux. 
    // Pour équilibrer linéaire/angulaire, on booste souvent l'angle ici.
    Vector2DAndRotation cmd_pid = error_P*P + error_I*I + error_D*D;

    // 7. Sortie commande (Linéaire et Angulaire directes)
    command.x_y.x = cmd_pid.x_y.x; // Vitesse Linéaire
    command.teta  = cmd_pid.teta * 5.0;  // Vitesse Angulaire (Boosté x5 car l'erreur en radians est petite)
    command.x_y.y = 0;

    command_limiter(1000); // Max PWM

    // Utilise Relative car la commande est déjà calculée en "Avance" / "Tourne"
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
    double speed_left  = cmd_linear - cmd_rot;
    double speed_right = cmd_linear + cmd_rot;

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