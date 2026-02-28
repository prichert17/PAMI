
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

// Paramètres
const int JOYSTICK_DEADZONE = 50;
const int MAX_MOTOR_SPEED = 1000;

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
          sendPosition(0.0f, 0.0f); // Envoie la commande pour rentrer à (0,0)
          Serial.println(">> MODE AUTO : Retour à la base (0,0) !");
        }
      }

      // ---------------------------------------------------------
      // 2. REPRISE MANUELLE & PILOTAGE
      // ---------------------------------------------------------
      // Si on touche aux joysticks, on annule l'auto et on repasse en manuel
      if (ly != 0 || rx != 0) {
        if (mode_auto) {
          mode_auto = false;
          setModeManuel();
          Serial.println(">> MODE MANUEL : Reprise du contrôle !");
        }
      }

      // On n'envoie les commandes moteurs QUE si on est en mode manuel
      if (!mode_auto) {
        // --- GESTION DE L'ACCÉLÉRATION (GÂCHETTE DROITE) ---
        int current_max_speed = 400; 
        
        // ctl->throttle() lit la gâchette droite R2/RT (de 0 à 1023)
        if (ctl->throttle() > 50) { 
            current_max_speed = MAX_MOTOR_SPEED; // Passe à 800
        }

        // --- CALCUL DES MOTEURS ---
        int16_t mix_m1 = rx + ly; 
        int16_t mix_m2 = rx - ly; 

        mix_m1 = constrain(mix_m1, -512, 512);
        mix_m2 = constrain(mix_m2, -512, 512);

        int16_t motor1 = map(mix_m1, -512, 512, -current_max_speed, current_max_speed);
        int16_t motor2 = map(mix_m2, -512, 512, -current_max_speed, current_max_speed);

        sendMotors(motor1, motor2);
      }
    }
  }
  
  delay(20);  // ~50Hz
}

