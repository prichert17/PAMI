#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- PINS DEFINITIONS ---
// UART STM32
#define PIN_RX2       16
#define PIN_TX2       17

// I2C SENSORS
#define PIN_SDA       21
#define PIN_SCL       22
// TOF XSHUT (LPN)
#define PIN_TOF1_LPN  4
#define PIN_TOF2_LPN  23
#define PIN_TOF3_LPN  18

// Actionneurs
#define PIN_SERVO_1   19
#define PIN_SERVO_2   15
#define PIN_SERVO_3   27
#define PIN_LED_DATA  14
#define PIN_MOT1_DIR1  32 // Moteur direct 1
#define PIN_MOT1_DIR2  33
#define PIN_MOT2_DIR1  25 // Moteur direct 2
#define PIN_MOT2_DIR2  26

// INPUTS
#define PIN_TIRETTE   39 // À définir (Pull-up interne requis)
#define PIN_SW_COLOR  34 // Input Only
#define PIN_SW_DEBUG  35 // Input Only
#define PIN_SW_MODE   36 // "UP" - Souvent VP/VN (Input Only)

// --- CONSTANTS ---
#define MATCH_DURATION_MS 99000 // 99 secondes (marge sécu)
#define BATTERY_MIN_V     3.0f  // Seuil alerte (par cellule, ou total à adapter)
#define BATTERY_TIMEOUT   1000  // Temps min sous le seuil pour déclencher l'erreur

// --- TOF GEOMETRY (Matriciels 8x8) ---
// Ordre des capteurs dans le code : [0]=TOF2(Gauche), [1]=TOF3(Centre), [2]=TOF1(Droite)
// On lit uniquement la ligne du milieu pour économiser la puissance GPU
// Angles en degrés, orientation : 0°=devant, +90°=gauche, -90°=droite (convention robot)

#define TOF_ANGLE_GAUCHE  -55.5f  // TOF2 : tourné 55.5° à gauche
#define TOF_FOV_GAUCHE    60.0f   // 60° de champ

#define TOF_ANGLE_CENTRE  0.0f    // TOF3 : au centre
#define TOF_FOV_CENTRE    45.0f   // 45° de champ

#define TOF_ANGLE_DROITE  55.5f   // TOF1 : tourné 55.5° à droite
#define TOF_FOV_DROITE    60.0f   // 60° de champ

// Paramètres de la matricielle : on lit seulement la ligne du milieu (8 zones par capteur)
#define TOF_ZONES_PER_LINE 8
#define TOF_LINE_MIDDLE    4      // Indice de la ligne du milieu dans la matrice 8x8

#endif