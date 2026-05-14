#include "asservissement.hpp"

// Paramètres stricts anti-patinage
namespace {
constexpr double kWheelCmdMax = 700.0;              // borne stricte PWM-like
constexpr double kRotCouplingStart = 180.0;         // commencer réduction de linéaire
constexpr double kRotCouplingMax = 600.0;           // rotation à partir de laquelle la réduction est maximale
constexpr double kLinearReductionAtMaxRot = 0.35;   // fraction à réduire au max

// Seuils de détection de patinage basés sur vitesses mesurées (mm/s)
constexpr double kSlipDiffThreshold = 140.0;        // différence entre vitesses mesurées
constexpr double kSlipMinFast = 120.0;              // la roue rapide doit dépasser ceci
constexpr double kSlipMinScale = 0.50;              // échelle minimale appliquée aux commandes
}

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

    // Réduction progressive du linéaire si rotation forte (préserve la rapidité en conduite normale)
    double rot_abs = fabs(cmd_rot);
    if (rot_abs > kRotCouplingStart) {
        double ratio = (rot_abs - kRotCouplingStart) / (kRotCouplingMax - kRotCouplingStart);
        if (ratio > 1.0) ratio = 1.0;
        double linear_scale = 1.0 - kLinearReductionAtMaxRot * ratio;
        if (linear_scale < 0.0) linear_scale = 0.0;
        cmd_linear *= linear_scale;
    }

    // Mixage roues
    double speed_left  = cmd_linear - cmd_rot;
    double speed_right = cmd_linear + cmd_rot;

    // Limitation homothétique (évite clamp roue par roue qui favorise le patinage)
    double max_abs_cmd = fabs(speed_left);
    if (fabs(speed_right) > max_abs_cmd) max_abs_cmd = fabs(speed_right);
    if (max_abs_cmd > kWheelCmdMax) {
        double scale = kWheelCmdMax / max_abs_cmd;
        speed_left *= scale;
        speed_right *= scale;
    }

    // Détection de patinage: si la différence de vitesses mesurées est grande
    // alors qu'une roue est rapide, on réduit les commandes pour retrouver l'adhérence.
    float v_left_meas = (*wheels)[0].get_speed() * CONSTANTS::WHEEL_RADIUS; // mm/s
    float v_right_meas = (*wheels)[1].get_speed() * CONSTANTS::WHEEL_RADIUS; // mm/s
    double abs_diff = fabs((double)v_left_meas - (double)v_right_meas);
    double fast = std::max(fabs((double)v_left_meas), fabs((double)v_right_meas));
    double slow = std::min(fabs((double)v_left_meas), fabs((double)v_right_meas));
    if (abs_diff > kSlipDiffThreshold && fast > kSlipMinFast && slow < fast * 0.7) {
        double scale = (slow + 40.0) / (fast + 40.0);
        if (scale < kSlipMinScale) scale = kSlipMinScale;
        speed_left *= scale;
        speed_right *= scale;
    }

    // Appliquer aux moteurs
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
    if (dist_to_target < 5.0) {  // 5mm de précision (était 20mm)
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
    static double straight_line_boost = 0.0;  // Boost progressif en ligne droite
    double threshold_enter = 0.4;  // ~23° pour passer en rotation
    double threshold_exit = 0.15;  // ~9° pour sortir de rotation

    if (fabs(angle_error) > threshold_enter) in_rotation_mode = true;
    else if (fabs(angle_error) < threshold_exit) in_rotation_mode = false;

    // Phase 1: Mode rotation
    if (in_rotation_mode) {
        double cmd_rot = error_P.teta * P + error_I.teta * I + error_D.teta * D;
        // Boost initial pour vaincre l'inertie
        double min_cmd = 100.0;
        if (fabs(cmd_rot) < min_cmd && fabs(angle_error) > 0.05)
            cmd_rot = (cmd_rot > 0) ? min_cmd : -min_cmd;
        command.teta = cmd_rot * 3;
        command.x_y.x = 0;
        // Reset du boost en ligne droite dès qu'on entre en rotation
        straight_line_boost = 0.0;
    } 
    // Phase 2: Bien orienté -> avancer
    else {
        /*
        // Accélération progressive du boost en ligne droite
        double max_boost = 300.0;  // Vitesse supplémentaire max
        double boost_rate = 5.0;   // Incrément par cycle (100Hz -> ~3s pour atteindre max)
        double angle_tolerance = 0.08;  // ~4.5° - seuil pour considérer l'angle comme correct
        
        // N'augmenter le boost que si l'angle est vraiment correct
        if (fabs(angle_error) < angle_tolerance && straight_line_boost < max_boost) {
            straight_line_boost += boost_rate;
            if (straight_line_boost > max_boost) straight_line_boost = max_boost;
        }*/
        double slow_down_distance = 100.0;  // Ralentir à partir de 100mm
        double speed_factor = (dist_to_target < slow_down_distance) 
            ? (dist_to_target / slow_down_distance) 
            : 1.0;
        command.x_y.x = (error_P.x_y.x * P + straight_line_boost) * speed_factor;
        command.teta = error_P.teta * P * 40.0;
    }
    command.x_y.y = 0;

    // Limite seulement le linéaire, pas l'angle
    if (command.x_y.x > 700.0) command.x_y.x = 700.0;
    if (command.x_y.x < -700.0) command.x_y.x = -700.0;

    // Rampe d'accélération (limite variation par cycle)
    static Vector2DAndRotation last_cmd(0, 0, 0);
    double max_delta = 12.0;  // Max variation par cycle (plus conservatif pour éviter décrochement)
    if (command.x_y.x - last_cmd.x_y.x > max_delta) command.x_y.x = last_cmd.x_y.x + max_delta;
    if (command.x_y.x - last_cmd.x_y.x < -max_delta) command.x_y.x = last_cmd.x_y.x - max_delta;
    if (command.teta - last_cmd.teta > max_delta) command.teta = last_cmd.teta + max_delta;
    if (command.teta - last_cmd.teta < -max_delta) command.teta = last_cmd.teta - max_delta;
    last_cmd = command;

    set_motors_power_relative(command);
}

void Asserv_Position::stop_asserv(){
    asserv_started = false;
    // Reset de l'intégrateur pour éviter un saut au redémarrage
    error_I = Vector2DAndRotation(0, 0, 0);
}

void Asserv_Position::set_target_position(Vector2DAndRotation target_position){
    this->target_position = target_position;
    // Reset de l'intégrateur pour éviter un biais vers l'ancienne direction
    error_I = Vector2DAndRotation(0, 0, 0);
}

void Asserv_Position::set_motors_power_relative(Vector2DAndRotation power){
    double cmd_linear = power.x_y.x;
    double cmd_rot    = power.teta;

    // Réduction progressive du linéaire si rotation forte
    double rot_abs = fabs(cmd_rot);
    if (rot_abs > kRotCouplingStart) {
        double ratio = (rot_abs - kRotCouplingStart) / (kRotCouplingMax - kRotCouplingStart);
        if (ratio > 1.0) ratio = 1.0;
        double linear_scale = 1.0 - kLinearReductionAtMaxRot * ratio;
        if (linear_scale < 0.0) linear_scale = 0.0;
        cmd_linear *= linear_scale;
    }

    double speed_left  = cmd_linear - cmd_rot;
    double speed_right = cmd_linear + cmd_rot;

    // Limitation homothétique
    double max_abs_cmd = fabs(speed_left);
    if (fabs(speed_right) > max_abs_cmd) max_abs_cmd = fabs(speed_right);
    if (max_abs_cmd > kWheelCmdMax) {
        double scale = kWheelCmdMax / max_abs_cmd;
        speed_left *= scale;
        speed_right *= scale;
    }

    // Détection de patinage et mitigation
    float v_left_meas = (*wheels)[0].get_speed() * CONSTANTS::WHEEL_RADIUS;
    float v_right_meas = (*wheels)[1].get_speed() * CONSTANTS::WHEEL_RADIUS;
    double abs_diff = fabs((double)v_left_meas - (double)v_right_meas);
    double fast = std::max(fabs((double)v_left_meas), fabs((double)v_right_meas));
    double slow = std::min(fabs((double)v_left_meas), fabs((double)v_right_meas));
    if (abs_diff > kSlipDiffThreshold && fast > kSlipMinFast && slow < fast * 0.7) {
        double scale = (slow + 40.0) / (fast + 40.0);
        if (scale < kSlipMinScale) scale = kSlipMinScale;
        speed_left *= scale;
        speed_right *= scale;
    }

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