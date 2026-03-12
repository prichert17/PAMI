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

// --- VARIABLES D'ACCÉLÉRATION ET ANTI-PATINAGE ---
float current_motor1 = 0;
float current_motor2 = 0;
const float MAX_ACCEL_STEP = 40.0; // Accélération globale des moteurs

float current_smoothed_rx = 0.0f; 
const float MAX_ROTATION_STEP = 0.04f; // <-- NOUVEAU : Vitesse de mise en virage (0.01 = très lent, 0.1 = rapide)

// Variables pour les Servomoteurs
Servo servo1;
Servo servo2;
int angleServo1 = 63; // Position initiale
int angleServo2 = 170;

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
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(PIN_SERVO_1, 500, 2400);
  servo2.attach(PIN_SERVO_2, 500, 2400);
  
  servo1.write(angleServo1);
  servo2.write(angleServo2);

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

      // --- CONTRÔLE DES SERVOS (Sychronisés et inversés) ---
        // La gâchette droite (RT / Throttle) ferme/descend le mécanisme
        if (ctl->brake() > 50) {
            angleServo1 -= 2; 
            angleServo2 += 2; // Mouvement inversé
        }
        
        // Le bouton droit (RB / R1) ouvre/monte le mécanisme
        if (ctl->l1()) {
            angleServo1 += 2;
            angleServo2 -= 2; // Mouvement inversé
        }

        // Sécurité pour ne pas forcer les butées mécaniques
        angleServo1 = constrain(angleServo1, 63, 180);
        angleServo2 = constrain(angleServo2, 0, 170);

        // Envoi des commandes
        servo1.write(angleServo1);
        servo2.write(angleServo2);

// --- CONTRÔLE DES MOTEURS ESP32 (Synchronisés et inversés) ---
        // Gâchette gauche (LT / Brake) : Fait tourner le mécanisme dans un sens
        if (ctl->throttle() > 50) {
            // Moteur 1 : Tourne en "Avant"
            digitalWrite(PIN_MOT1_DIR1, HIGH);
            digitalWrite(PIN_MOT1_DIR2, LOW);
            // Moteur 2 : Tourne en "Arrière" (Inversé)
            digitalWrite(PIN_MOT2_DIR1, HIGH);
            digitalWrite(PIN_MOT2_DIR2, LOW);
        } 
        // Bouton gauche (LB / L1) : Fait tourner le mécanisme dans l'autre sens
        else if (ctl->r1()) {
            // Moteur 1 : Tourne en "Arrière"
            digitalWrite(PIN_MOT1_DIR1, LOW);
            digitalWrite(PIN_MOT1_DIR2, HIGH);
            // Moteur 2 : Tourne en "Avant" (Inversé)
            digitalWrite(PIN_MOT2_DIR1, LOW);
            digitalWrite(PIN_MOT2_DIR2, HIGH);
        } 
        // Si on ne touche à la main gauche : Arrêt complet des moteurs
        else {
            digitalWrite(PIN_MOT1_DIR1, LOW);
            digitalWrite(PIN_MOT1_DIR2, LOW);
            digitalWrite(PIN_MOT2_DIR1, LOW);
            digitalWrite(PIN_MOT2_DIR2, LOW);
        }

        // --- COURBES ET ANTI-PATINAGE EN ROTATION ---
        float target_ly = (float)ly / 512.0f;
        float target_rx = (float)rx / 512.0f;
        
        target_ly = target_ly * target_ly * target_ly;
        target_rx = target_rx * target_rx * target_rx; // Commande cible de rotation

        // NOUVEAU : Rampe de lissage pour la rotation (évite le dérapage)
        if (target_rx > current_smoothed_rx) {
          current_smoothed_rx = min(target_rx, current_smoothed_rx + MAX_ROTATION_STEP);
        } else {
          current_smoothed_rx = max(target_rx, current_smoothed_rx - MAX_ROTATION_STEP);
        }

        // On calcule la priorité à la direction avec la valeur lissée
        float adjusted_ly = target_ly * (1.0 - abs(current_smoothed_rx) * 0.7);

        // Mixage Arcade
        float target_m1 = (current_smoothed_rx + adjusted_ly) * current_max_speed;
        float target_m2 = (current_smoothed_rx - adjusted_ly) * current_max_speed;

        target_m1 = constrain(target_m1, -current_max_speed, current_max_speed);
        target_m2 = constrain(target_m2, -current_max_speed, current_max_speed);

        // --- RAMPE D'ACCÉLÉRATION FINALE (Anti-Voltage Drop) ---
        if (target_m1 > current_motor1) {
          current_motor1 = min((float)target_m1, current_motor1 + MAX_ACCEL_STEP);
        } else {
          current_motor1 = max((float)target_m1, current_motor1 - MAX_ACCEL_STEP);
        }

        if (target_m2 > current_motor2) {
          current_motor2 = min((float)target_m2, current_motor2 + MAX_ACCEL_STEP);
        } else {
          current_motor2 = max((float)target_m2, current_motor2 - MAX_ACCEL_STEP);
        }

        // Envoi à la STM32
        sendMotors((int16_t)current_motor1, (int16_t)current_motor2);
      }
    }
  }
  
  delay(20);  // ~50Hz
}