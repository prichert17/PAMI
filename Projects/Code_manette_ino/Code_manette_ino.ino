
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
const int MAX_MOTOR_SPEED = 800;

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
  receiveFromSTM32();

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerPtr ctl = myControllers[i];
    if (ctl && ctl->isConnected()) {
      // Lecture joystick gauche
      int ly = ctl->axisY();   // Avant/Arrière (-512 à 512)
      int rx = ctl->axisRX();  // Rotation (-512 à 512)

// Zone morte
      if (abs(ly) < JOYSTICK_DEADZONE) ly = 0;
      if (abs(rx) < JOYSTICK_DEADZONE) rx = 0;

      // --- CORRECTION DES AXES ET SÉCURITÉ ---
      // On modifie l'équation pour que 'ly' fasse avancer/reculer et 'rx' fasse tourner.
      int16_t mix_m1 = rx + ly; 
      int16_t mix_m2 = rx - ly; // L'inversion magique se fait ici !

      // Sécurité : on empêche la valeur de dépasser les limites -512 / 512
      // (Si vous poussez les deux joysticks à fond, rx+ly peut valoir 1024, ce qui fausserait le map() !)
      mix_m1 = constrain(mix_m1, -512, 512);
      mix_m2 = constrain(mix_m2, -512, 512);

      // Calcul des vitesses finales
      int16_t motor1 = map(mix_m1, -512, 512, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      int16_t motor2 = map(mix_m2, -512, 512, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

      // Envoi à la STM32
      sendMotors(motor1, motor2);
    }
  }
  
  delay(20);  // ~50Hz
}

