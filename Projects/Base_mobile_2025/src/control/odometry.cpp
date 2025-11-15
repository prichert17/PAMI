#include "odometry.hpp"


Odometry::Odometry(std::array<Wheel, 3> wheels):
    wheels(wheels),
    serial(NULL),
    last_ticks_droit(0),
    last_ticks_gauche(0),
    mode_differentiel(false),
    position_now(Vector2DAndRotation(0.0, 0.0, 0.0)),
    position_last(Vector2DAndRotation(0.0, 0.0, 0.0)),
    speed_now(Vector2DAndRotation(0.0, 0.0, 0.0)),
    speed_filtered(Vector2DAndRotation(0.0, 0.0, 0.0))
{
    HAL_TIM_Base_Start_IT(&htim6);
}

void Odometry::set_mode_differentiel(bool enable) {
    mode_differentiel = enable;
    if (enable) {
        // Initialiser les dernières positions des encodeurs
        last_ticks_droit = wheels[0].get_nb_step_now();   // Moteur 1 (Droit)
        last_ticks_gauche = wheels[1].get_nb_step_now();  // Moteur 2 (Gauche)
    }
}

void Odometry::update_odometry_differentielle(){
    // Odométrie différentielle pour base à 2 roues
    
    // 1. Lire les ticks des deux moteurs
    int32_t ticks_droit = wheels[0].get_nb_step_now();   // Moteur 1 (TIM1) - Roue Droite
    int32_t ticks_gauche = wheels[1].get_nb_step_now();  // Moteur 2 (TIM2) - Roue Gauche
    
    // 2. Calculer le delta en ticks
    int32_t delta_ticks_D = ticks_droit - last_ticks_droit;
    int32_t delta_ticks_G = ticks_gauche - last_ticks_gauche;
    
    // 3. Convertir en distance (mm) avec correction odométrique
    double delta_D = delta_ticks_D * CONSTANTS::DISTANCE_PAR_TICK; //* CONSTANTS::ODOMETRY_CORRECTION_XY;
    double delta_G = delta_ticks_G * CONSTANTS::DISTANCE_PAR_TICK; //* CONSTANTS::ODOMETRY_CORRECTION_XY;
    
    // 4. Sauvegarder les positions actuelles
    last_ticks_droit = ticks_droit;
    last_ticks_gauche = ticks_gauche;
    
    // 5. Calculer le déplacement
    double delta_dist = (delta_D + delta_G) / 2.0;
    double delta_theta = (delta_D - delta_G) / CONSTANTS::ENTRE_AXE;
    
    // 6. Mettre à jour la pose (intégration avec approximation au milieu de l'arc)
    position_last = position_now;
    position_now.x_y.x += delta_dist * cos(position_now.teta + delta_theta / 2.0);
    position_now.x_y.y += delta_dist * sin(position_now.teta + delta_theta / 2.0);
    position_now.teta += delta_theta;
    
    // 7. Normaliser theta entre -PI et PI
    while (position_now.teta > 3.14159265358979323846) position_now.teta -= 2.0 * 3.14159265358979323846;
    while (position_now.teta < -3.14159265358979323846) position_now.teta += 2.0 * 3.14159265358979323846;
    
    // 8. Calculer la vitesse du robot
    speed_now = (position_now - position_last) * CONSTANTS::ODOMETRY_FREQ;
    
    // 9. Filtrer la vitesse
    speed_filter.add_elem(speed_now);
    speed_filtered = speed_filter.get_average();
}

void Odometry::update_odometry(){
    // Choisir la méthode d'odométrie selon le mode
    if (mode_differentiel) {
        update_odometry_differentielle();
        return;
    }
    
    // ========== MODE HOLONOME (ancien code conservé) ==========
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
