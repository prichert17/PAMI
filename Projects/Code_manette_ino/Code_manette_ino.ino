// Code pour l'ESP32 - Contrôle robot PAMI par manette
#include <Arduino.h>
#include "pami_com.h"
#include <Bluepad32.h>

// ============================================
// VARIABLES GLOBALES
// ============================================
bool mode_auto = false;
float target_x = 0.0f, target_y = 0.0f;
float current_x = 0.0f, current_y = 0.0f;

// Manette Bluetooth
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Paramètres de pilotage
const int JOYSTICK_DEADZONE = 50;
const int MAX_MOTOR_SPEED = 1000;

// Variables pour l'accélération progressive (Anti-Drop Tension)
float current_motor1 = 0;
float current_motor2 = 0;
const float MAX_ACCEL_STEP = 40.0;

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
  delay(5);
  resetSTM32();
  Serial.println("En attente de manette...");
}

// ============================================
// LOOP
// ============================================
void loop() {
  BP32.update();
  receiveFromSTM32(); // Met à jour current_x et current_y

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerPtr ctl = myControllers[i];
    if (ctl && ctl->isConnected()) {
      
      // Lecture des joysticks
      int ly = ctl->axisY();   // Avant/Arrière
      int rx = ctl->axisRX();  // Rotation

      // Zone morte des joysticks
      if (abs(ly) < JOYSTICK_DEADZONE) ly = 0;
      if (abs(rx) < JOYSTICK_DEADZONE) rx = 0;

      // ---------------------------------------------------------
      // 1. BOUTON RETURN TO HOME (Bouton Y / Triangle)
      // ---------------------------------------------------------
      if (ctl->y()) {
        if (!mode_auto) {
          mode_auto = true;
          setModeAuto();
          delay(5);
          sendPosition(0.0f, 0.0f);
          Serial.println(">> MODE AUTO : Retour à la base (0,0) !");
        }
      }

      // ---------------------------------------------------------
      // 2. REPRISE MANUELLE & PILOTAGE
      // ---------------------------------------------------------
      if (ly != 0 || rx != 0) {
        if (mode_auto) {
          mode_auto = false;
          setModeManuel();
          Serial.println(">> MODE MANUEL : Reprise du contrôle !");
        }
      }

      if (!mode_auto) {
        // --- 1. GÂCHETTE PROPORTIONNELLE ---
        int trigger_val = ctl->throttle();
        float current_max_speed = map(trigger_val, 0, 1023, 400, MAX_MOTOR_SPEED);

        // --- 2. COURBES ET PRIORITÉ À LA DIRECTION ---
        float norm_ly = (float)ly / 512.0f;
        float norm_rx = (float)rx / 512.0f;
        
        norm_ly = norm_ly * norm_ly * norm_ly;
        norm_rx = norm_rx * norm_rx * norm_rx;

        // LA MAGIE EST ICI : Plus on tourne fort, plus on réduit la force d'avancement pure.
        // Le "0.7" détermine l'agressivité du virage (0.0 = roue intérieure arrêtée, 1.0 = marche arrière violente)
        float adjusted_ly = norm_ly * (1.0 - abs(norm_rx) * 0.7);

        // --- CALCUL CIBLE (Arcade Drive) ---
        float target_m1 = (norm_rx + adjusted_ly) * current_max_speed;
        float target_m2 = (norm_rx - adjusted_ly) * current_max_speed;

        target_m1 = constrain(target_m1, -current_max_speed, current_max_speed);
        target_m2 = constrain(target_m2, -current_max_speed, current_max_speed);

        // --- 3. RAMPE D'ACCÉLÉRATION (Anti-Voltage Drop) ---
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