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
#define PIN_TIRETTE   36 // À définir (Pull-up interne requis)
#define PIN_SW_COLOR  34 // Input Only
#define PIN_SW_DEBUG  35 // Input Only
#define PIN_SW_MODE   36 // "UP" - Souvent VP/VN (Input Only)

// --- CONSTANTS ---
#define MATCH_DURATION_MS 99000 // 99 secondes (marge sécu)
#define BATTERY_MIN_V     3.0f  // Seuil alerte (par cellule, ou total à adapter)
#define BATTERY_TIMEOUT   1000  // Temps min sous le seuil pour déclencher l'erreur

#endif