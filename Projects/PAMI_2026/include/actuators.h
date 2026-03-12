#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

// ============================================
// OBJETS SERVOS
// ============================================
static Servo servo1;
static Servo servo2;
static Servo servo3;

// ============================================
// CONFIGURATION MOTEURS DC
// ============================================
#define MOTOR_PWM_FREQ  20000  // 20kHz pour éviter le bruit audible
#define MOTOR_PWM_RES   8      // 8 bits (0-255)

// Canaux LEDC pour les moteurs (les servos utilisent les canaux 0-2 via ESP32Servo)
#define LEDC_CH_MOT1_DIR1  4
#define LEDC_CH_MOT1_DIR2  5
#define LEDC_CH_MOT2_DIR1  6
#define LEDC_CH_MOT2_DIR2  7

// ============================================
// FONCTIONS SERVOS
// ============================================

/**
 * Initialise les 3 servos
 */
inline void initServos() {
    servo1.attach(PIN_SERVO_1);
    servo2.attach(PIN_SERVO_2);
    servo3.attach(PIN_SERVO_3);
}

/**
 * Positionne un servo à un angle donné (0-180°)
 * @param servo Numéro du servo (1, 2 ou 3)
 * @param angle Angle en degrés (0-180)
 */
inline void setServoAngle(uint8_t servo, uint8_t angle) {
    switch (servo) {
        case 1: servo1.write(angle); break;
        case 2: servo2.write(angle); break;
        case 3: servo3.write(angle); break;
    }
}

/**
 * Positionne un servo avec une valeur en microsecondes (500-2500µs)
 * @param servo Numéro du servo (1, 2 ou 3)
 * @param us Pulse en microsecondes
 */
inline void setServoMicroseconds(uint8_t servo, uint16_t us) {
    switch (servo) {
        case 1: servo1.writeMicroseconds(us); break;
        case 2: servo2.writeMicroseconds(us); break;
        case 3: servo3.writeMicroseconds(us); break;
    }
}

/**
 * Désactive un servo (libère le signal PWM)
 */
inline void disableServo(uint8_t servo) {
    switch (servo) {
        case 1: servo1.detach(); break;
        case 2: servo2.detach(); break;
        case 3: servo3.detach(); break;
    }
}

// ============================================
// FONCTIONS MOTEURS DC
// ============================================

/**
 * Initialise les 2 moteurs DC (pont H avec PWM)
 */
inline void initMotors() {
    // Moteur 1
    ledcSetup(LEDC_CH_MOT1_DIR1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(PIN_MOT1_DIR1, LEDC_CH_MOT1_DIR1);
    ledcSetup(LEDC_CH_MOT1_DIR2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(PIN_MOT1_DIR2, LEDC_CH_MOT1_DIR2);
    // Moteur 2
    ledcSetup(LEDC_CH_MOT2_DIR1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(PIN_MOT2_DIR1, LEDC_CH_MOT2_DIR1);
    ledcSetup(LEDC_CH_MOT2_DIR2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(PIN_MOT2_DIR2, LEDC_CH_MOT2_DIR2);
    
    // Arrêt initial
    ledcWrite(LEDC_CH_MOT1_DIR1, 0);
    ledcWrite(LEDC_CH_MOT1_DIR2, 0);
    ledcWrite(LEDC_CH_MOT2_DIR1, 0);
    ledcWrite(LEDC_CH_MOT2_DIR2, 0);
}

/**
 * Contrôle un moteur DC
 * @param motor Numéro du moteur (1 ou 2)
 * @param speed Vitesse de -255 à +255 (négatif = arrière)
 */
inline void setMotorSpeed(uint8_t motor, int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    
    uint8_t pwm = abs(speed);
    
    if (motor == 1) {
        if (speed > 0) {
            ledcWrite(LEDC_CH_MOT1_DIR1, pwm);
            ledcWrite(LEDC_CH_MOT1_DIR2, 0);
        } else if (speed < 0) {
            ledcWrite(LEDC_CH_MOT1_DIR1, 0);
            ledcWrite(LEDC_CH_MOT1_DIR2, pwm);
        } else {
            ledcWrite(LEDC_CH_MOT1_DIR1, 0);
            ledcWrite(LEDC_CH_MOT1_DIR2, 0);
        }
    } else if (motor == 2) {
        if (speed > 0) {
            ledcWrite(LEDC_CH_MOT2_DIR1, pwm);
            ledcWrite(LEDC_CH_MOT2_DIR2, 0);
        } else if (speed < 0) {
            ledcWrite(LEDC_CH_MOT2_DIR1, 0);
            ledcWrite(LEDC_CH_MOT2_DIR2, pwm);
        } else {
            ledcWrite(LEDC_CH_MOT2_DIR1, 0);
            ledcWrite(LEDC_CH_MOT2_DIR2, 0);
        }
    }
}

/**
 * Contrôle les deux moteurs simultanément (pour conduite différentielle)
 * @param speedLeft Vitesse moteur gauche (-255 à +255)
 * @param speedRight Vitesse moteur droite (-255 à +255)
 */
inline void setMotors(int16_t speedLeft, int16_t speedRight) {
    setMotorSpeed(1, speedLeft);
    setMotorSpeed(2, speedRight);
}


#endif
