#include "motor.hpp"
#include <cmath>
#include <stdlib.h> // pour abs()

// Constructeur
Wheel::Wheel(uint8_t id, float encoder_steps, float radius, float angle)
    : wheel_id(id), encoder_step_per_rev(encoder_steps), 
      wheel_radius(radius), angle_offset(angle),
      last_encoder_count(0), current_speed(0.0f)
{
    // Attribution des timers selon l'ID de la roue
    if (id == 1) {
        encoder_timer = &htim2;  // 32 bits
        pwm_timer = &htim1;
        pwm_channel = TIM_CHANNEL_1;
    } else if (id == 2) {
        encoder_timer = &htim3;  // 16 bits !
        pwm_timer = &htim16;
        pwm_channel = TIM_CHANNEL_1;
    } else if (id == 3) {
        encoder_timer = nullptr; // Pas d'encodeur
        pwm_timer = &htim8;
        pwm_channel = TIM_CHANNEL_1;
    }
}

// Pilotage PWM en mode "Locked Anti-Phase"
// duty_cycle : entre -1000 (Recule max) et +1000 (Avance max)
void Wheel::set_pwm(int16_t duty_cycle) {
    
    // 1. Limitation de sécurité (Clamping)
    if (duty_cycle > 1000) duty_cycle = 1000;
    if (duty_cycle < -1000) duty_cycle = -1000;

    // 2. Récupération dynamique de l'ARR (votre valeur 8499)
    // Cela rend le code compatible peu importe la fréquence PWM choisie dans CubeMX
    uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(pwm_timer);

    // 3. Calcul du CCR (Capture Compare Register)
    // Transformation : [-1000 ... 0 ... +1000]  ---> [0 ... ARR/2 ... ARR]
    // Exemple avec ARR = 8499 :
    // -1000 -> 0%   (0)
    //     0 -> 50%  (4249) -> Moteur à l'arrêt (freiné)
    // +1000 -> 100% (8499)
    
    int32_t ccr_val = ((int32_t)duty_cycle + 1000) * autoreload / 2000;

    // 4. Application sur le canal (Le canal N est géré automatiquement par le hardware)
    __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, ccr_val);
}

// Lecture brute du compteur encodeur
int32_t Wheel::get_encoder_count() {
    if (encoder_timer == nullptr) return 0;
    
    // On récupère la valeur brute du compteur
    return (int32_t)(__HAL_TIM_GET_COUNTER(encoder_timer));
}

// Calcul de la vitesse avec gestion du débordement
void Wheel::update_speed(float dt) {
    if (encoder_timer == nullptr) return;

    int32_t current_count = get_encoder_count();
    int32_t delta = current_count - last_encoder_count;

    // --- GESTION DU DÉBORDEMENT (OVERFLOW) ---
    // Pour TIM3 (16 bits), le compteur boucle à 65535.
    // Si on passe de 65530 à 10 (marche avant), le delta brut est -65520 (Faux !)
    // Il faut corriger pour trouver le vrai delta (+16)
    
    if (encoder_timer->Instance == TIM3 || encoder_timer->Instance == TIM4) {
        // Seuil fixé à ~ la moitié de 65536. 
        // Si le saut est trop grand, c'est qu'on a fait un tour de compteur.
        if (delta > 32000) {
            delta -= 65536; // Correction marche arrière
        }
        else if (delta < -32000) {
            delta += 65536; // Correction marche avant
        }
    }
    // Note : Pour TIM2 (32 bits), le débordement est si lointain qu'on l'ignore souvent,
    // ou on applique la même logique avec les limites 32 bits si nécessaire.

    last_encoder_count = current_count;
    
    // Calcul de la vitesse en rad/s
    // (delta / pas_par_tour) * 2*PI / temps_écoulé
    current_speed = ((float)delta / encoder_step_per_rev) * (2.0f * M_PI) / dt;
}

float Wheel::get_speed() {
    return current_speed;
}