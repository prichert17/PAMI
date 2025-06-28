#include <Arduino.h>
#include <ESP32Servo.h>
// Déclaration des variables supplémentaires pour la position du robot




// Déclaration des fonctions
void countPulse1();
void countPulse2();
void set_vitesse(float cmd1, float cmd2);
void computePID();
void computePosition(float deltaLeft, float deltaRight);
void deplacement();
void deplacement_angulaire_horaire();
void deplacement_angulaire_antihoraire();
void print_vitesse();
void go_to();
void updateServo();
void asser_position_setup();
void asser_position_loop(bool stop);
bool arrivee_destination();
bool arrivee_destination_finale();
void onServoTimer(); // Fonction de gestion du servo
void setupServoInterrupt(); // Fonction de configuration du timer pour le servo
