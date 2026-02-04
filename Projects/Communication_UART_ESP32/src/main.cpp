
// Code pour l'ESP32 - Contrôle robot PAMI
#include "pami_com.h"

// ============================================
// VARIABLES GLOBALES
// ============================================
bool mode_auto = false;  // false = MANUEL, true = AUTO
float target_x = 0.0f, target_y = 0.0f;
float current_x = 0.0f, current_y = 0.0f;

// Liste de waypoints
struct Point { float x; float y; };
Point waypoints[] = {
  {1000, 0},
  {1000, 1000},
  {0, 1000},
  {0, 0}
};
const int NB_WAYPOINTS = sizeof(waypoints) / sizeof(waypoints[0]);
int currentWaypoint = 0;

// Timing
const float TOLERANCE = 50.0f;  // mm
unsigned long reachedTime = 0;
bool isNearTarget = false;

// ============================================
// SETUP
// ============================================
void setup() {
  initPAMI();
  delay(100);
  setModeAuto();
}

// ============================================
// LOOP
// ============================================
int step = 0;
unsigned long timer = 0;

void sendNextWaypoint() {
  if (currentWaypoint < NB_WAYPOINTS) {
    target_x = waypoints[currentWaypoint].x;
    target_y = waypoints[currentWaypoint].y;
    sendPosition(target_x, target_y);
    Serial.printf(">> Waypoint %d: X:%.0f Y:%.0f\n", currentWaypoint, target_x, target_y);
    isNearTarget = false;
  } else {
    Serial.println(">> Parcours terminé");
  }
}

void loop() {
  receiveFromSTM32();
  
  if (step == 0 && millis() > 500) {
    resetSTM32();
    step = 1;
    timer = millis();
  }
  else if (step == 1 && millis() - timer > 2000) {
    sendNextWaypoint();
    step = 2;
  }
  else if (step == 2 && currentWaypoint < NB_WAYPOINTS) {
    if (checkPosition(TOLERANCE)) {
      if (!isNearTarget) {
        isNearTarget = true;
        reachedTime = millis();
        Serial.printf(">> Position atteinte (X:%.0f Y:%.0f)\n", current_x, current_y);
      }
      else if (millis() - reachedTime > 50) {
        currentWaypoint++;
        sendNextWaypoint();
      }
    } else {
      isNearTarget = false;
    }
  }
}

