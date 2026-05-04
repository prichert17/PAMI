// Code pour l'ESP32 - Contrôle robot PAMI par manette
#include <Arduino.h>
#include "pami_com.h"
#include <Bluepad32.h>
#include <ESP32Servo.h>

// ============================================
// DÉFINITION DES PINS (Actionneurs supplémentaires)
// ============================================
#define PIN_SERVO_1   19
#define PIN_SERVO_2   15
#define PIN_SERVO_3   27  
#define PIN_LED_DATA  14

// Moteurs directs ESP32
#define PIN_MOT1_DIR1  32
#define PIN_MOT1_DIR2  33
#define PIN_MOT2_DIR1  25
#define PIN_MOT2_DIR2  26

// ============================================
// VARIABLES GLOBALES
// ============================================
bool mode_auto = false;
float target_x = 0.0f, target_y = 0.0f;
float current_x = 0.0f, current_y = 0.0f;

// Manette Bluetooth
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Paramètres de pilotage principal
const int JOYSTICK_DEADZONE = 50;
const int MAX_MOTOR_SPEED = 1000;

// --- VARIABLES D'ACCÉLÉRATION ET ANTI-PATINAGE MOTEURS ---
float current_motor1 = 0;
float current_motor2 = 0;
const float MAX_ACCEL_STEP = 40.0;

float current_smoothed_rx = 0.0f; 
const float MAX_ROTATION_STEP = 0.04f;

// ============================================
// VARIABLES SERVOMOTEURS (MODIFIÉ)
// ============================================
Servo servo1;
Servo servo2;
Servo servo3;

// 1. Angles Cibles (Ce que tu demandes avec la manette)
float targetServo1 = 63.0; 
float targetServo2 = 170.0;
float targetServo3 = 0.0; 

// 2. Angles Actuels (La position réelle lissée)
float currentServo1 = 63.0; 
float currentServo2 = 170.0;
float currentServo3 = 0.0;

// 3. VITESSE MAX DES SERVOS (Le paramètre qui sauve ta batterie !)
// 0.8 degré par boucle (20ms) = mouvement doux. Baisse à 0.5 si l'alim coupe encore.
const float SERVO_MAX_STEP = 0.8; 

// ============================================
// CALLBACKS MANETTE
// ============================================
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Manette connectée à l'index %d\n", i);
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Manette déconnectée de l'index %d\n", i);
      break;
    }
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  initPAMI();
  BP32.setup(&onConnectedController, &onDisconnectedController);
  setModeManuel();
  
  // --- INITIALISATION DES SERVOS ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  
  servo1.attach(PIN_SERVO_1, 500, 2400);
  servo2.attach(PIN_SERVO_2, 500, 2400);
  servo3.attach(PIN_SERVO_3, 500, 2400);
  
  // On écrit la position initiale
  servo1.write((int)currentServo1);
  servo2.write((int)currentServo2);
  servo3.write((int)currentServo3);

  // --- INITIALISATION DES MOTEURS DIRECTS ---
  pinMode(PIN_MOT1_DIR1, OUTPUT);
  pinMode(PIN_MOT1_DIR2, OUTPUT);
  pinMode(PIN_MOT2_DIR1, OUTPUT);
  pinMode(PIN_MOT2_DIR2, OUTPUT);
  pinMode(PIN_LED_DATA, OUTPUT);

  digitalWrite(PIN_MOT1_DIR1, LOW);
  digitalWrite(PIN_MOT1_DIR2, LOW);
  digitalWrite(PIN_MOT2_DIR1, LOW);
  digitalWrite(PIN_MOT2_DIR2, LOW);

  delay(5);
  resetSTM32();
  Serial.println("En attente de manette...");
}

// ============================================
// LOOP
// ============================================
void loop() {
  BP32.update();
  receiveFromSTM32(); 

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerPtr ctl = myControllers[i];
    if (ctl && ctl->isConnected()) {
      
      int ly = ctl->axisY();   // Avant/Arrière
      int rx = ctl->axisRX();  // Rotation

      if (abs(ly) < JOYSTICK_DEADZONE) ly = 0;
      if (abs(rx) < JOYSTICK_DEADZONE) rx = 0;

      // 1. BOUTON RETURN TO HOME (Bouton X)
      if (ctl->x()) {
        if (!mode_auto) {
          mode_auto = true;
          setModeAuto();
          delay(5);
          sendPosition(0.0f, 0.0f);
          Serial.println(">> MODE AUTO : Retour à la base (0,0) !");
        }
      }

      // 2. REPRISE MANUELLE & PILOTAGE
      if (ly != 0 || rx != 0) {
        if (mode_auto) {
          mode_auto = false;
          setModeManuel();
          Serial.println(">> MODE MANUEL : Reprise du contrôle !");
        }
      }

      if (!mode_auto) {
        // --- GESTION DU BOOST (Bouton Y) ---
        float current_max_speed = 400;
        if (ctl->y()) { 
            current_max_speed = MAX_MOTOR_SPEED; 
        }

        // --- MODIFICATION DES CIBLES SERVOS 1 & 2 (Main Gauche) ---
        if (ctl->l1()) { // LT
            targetServo1 -= 2.0; 
            targetServo2 += 2.0; // Réglé à 2.0 pour éviter les saccades
        }
        if (ctl->brake() > JOYSTICK_DEADZONE) { // LB (Correction de la gâchette !)
            targetServo1 += 2.0;
            targetServo2 -= 2.0; 
        }

        // --- MODIFICATION DE LA CIBLE SERVO 3 (Main Droite) ---
        if (ctl->r1()) { // RT 
            targetServo3 -= 2.0; 
        }
        if (ctl->throttle() > JOYSTICK_DEADZONE) { // RB (Correction de la gâchette !)
            targetServo3 += 2.0;
        }

        // Sécurité Butées sur les Cibles
        targetServo1 = constrain(targetServo1, 63.0, 180.0);
        targetServo2 = constrain(targetServo2, 0.0, 170.0);
        targetServo3 = constrain(targetServo3, 0.0, 180.0);

        // ========================================================
        // LE SOFT-START (LA MAGIE ANTI-COUPURE BATTERIE)
        // ========================================================
        if (targetServo1 > currentServo1) currentServo1 = min(targetServo1, currentServo1 + SERVO_MAX_STEP);
        else if (targetServo1 < currentServo1) currentServo1 = max(targetServo1, currentServo1 - SERVO_MAX_STEP);

        if (targetServo2 > currentServo2) currentServo2 = min(targetServo2, currentServo2 + SERVO_MAX_STEP);
        else if (targetServo2 < currentServo2) currentServo2 = max(targetServo2, currentServo2 - SERVO_MAX_STEP);

        if (targetServo3 > currentServo3) currentServo3 = min(targetServo3, currentServo3 + SERVO_MAX_STEP);
        else if (targetServo3 < currentServo3) currentServo3 = max(targetServo3, currentServo3 - SERVO_MAX_STEP);

        // --- ÉCRITURE ANTI-VIBRATION ---
        static int last_s1 = -1;
        static int last_s2 = -1;
        static int last_s3 = -1;
        
        int out_s1 = (int)currentServo1;
        int out_s2 = (int)currentServo2;
        int out_s3 = (int)currentServo3;

        if (out_s1 != last_s1) { servo1.write(out_s1); last_s1 = out_s1; }
        if (out_s2 != last_s2) { servo2.write(out_s2); last_s2 = out_s2; }
        if (out_s3 != last_s3) { servo3.write(out_s3); last_s3 = out_s3; }

        // --- CONTRÔLE DES MOTEURS ESP32 (Mode Tank sur D-PAD) ---
        uint8_t dpad = ctl->dpad();
        
        if (dpad & 0x01) { 
            digitalWrite(PIN_MOT1_DIR1, HIGH); digitalWrite(PIN_MOT1_DIR2, LOW);
            digitalWrite(PIN_MOT2_DIR1, HIGH); digitalWrite(PIN_MOT2_DIR2, LOW);
        } 
        else if (dpad & 0x02) { 
            digitalWrite(PIN_MOT1_DIR1, LOW); digitalWrite(PIN_MOT1_DIR2, HIGH);
            digitalWrite(PIN_MOT2_DIR1, LOW); digitalWrite(PIN_MOT2_DIR2, HIGH);
        } 
        else if (dpad & 0x08) { 
            digitalWrite(PIN_MOT1_DIR1, LOW); digitalWrite(PIN_MOT1_DIR2, HIGH);
            digitalWrite(PIN_MOT2_DIR1, HIGH); digitalWrite(PIN_MOT2_DIR2, LOW);
        } 
        else if (dpad & 0x04) { 
            digitalWrite(PIN_MOT1_DIR1, HIGH); digitalWrite(PIN_MOT1_DIR2, LOW);
            digitalWrite(PIN_MOT2_DIR1, LOW); digitalWrite(PIN_MOT2_DIR2, HIGH);
        } 
        else { 
            digitalWrite(PIN_MOT1_DIR1, LOW); digitalWrite(PIN_MOT1_DIR2, LOW);
            digitalWrite(PIN_MOT2_DIR1, LOW); digitalWrite(PIN_MOT2_DIR2, LOW);
        }

        // --- COURBES ET ANTI-PATINAGE EN ROTATION (Moteurs Principaux) ---
        float target_ly = (float)ly / 512.0f;
        float target_rx = (float)rx / 512.0f;
        
        target_ly = target_ly * target_ly * target_ly;
        target_rx = target_rx * target_rx * target_rx;

        if (target_rx > current_smoothed_rx) {
          current_smoothed_rx = min(target_rx, current_smoothed_rx + MAX_ROTATION_STEP);
        } else {
          current_smoothed_rx = max(target_rx, current_smoothed_rx - MAX_ROTATION_STEP);
        }

        float adjusted_ly = target_ly * (1.0 - abs(current_smoothed_rx) * 0.7);

        float target_m1 = (current_smoothed_rx + adjusted_ly) * current_max_speed;
        float target_m2 = (current_smoothed_rx - adjusted_ly) * current_max_speed;

        // CORRECTION MATÉRIELLE
        target_m1 = target_m1 * 0.83f; 

        target_m1 = constrain(target_m1, -current_max_speed, current_max_speed);
        target_m2 = constrain(target_m2, -current_max_speed, current_max_speed);

        // --- RAMPE D'ACCÉLÉRATION FINALE ---
        if (target_m1 > current_motor1) current_motor1 = min((float)target_m1, current_motor1 + MAX_ACCEL_STEP);
        else current_motor1 = max((float)target_m1, current_motor1 - MAX_ACCEL_STEP);

        if (target_m2 > current_motor2) current_motor2 = min((float)target_m2, current_motor2 + MAX_ACCEL_STEP);
        else current_motor2 = max((float)target_m2, current_motor2 - MAX_ACCEL_STEP);

        // Envoi à la STM32
        sendMotors((int16_t)current_motor1, (int16_t)current_motor2);
      }
    }
  }
  
  delay(20);  // ~50Hz
}