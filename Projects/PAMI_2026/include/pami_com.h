#ifndef PAMI_COM_H
#define PAMI_COM_H

#include <HardwareSerial.h>

// ============================================
// CONFIGURATION
// ============================================
#define RXD2 16
#define TXD2 17
#define SERIAL_BAUD 115200

// ============================================
// VARIABLES GLOBALES
// ============================================
extern bool mode_auto;
extern float target_x, target_y;
extern float current_x, current_y, current_theta;
extern bool debugMode; // Flag pour désactiver les logs STM32

// ============================================
// INITIALISATION
// ============================================
inline void initPAMI() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("\n=== PAMI ESP32 Control ===\n");
}

// ============================================
// FONCTIONS D'ENVOI UART
// ============================================
inline void sendToSTM32(const char* cmd) {
  Serial2.println(cmd);
}

inline void sendMotors(int16_t m1, int16_t m2) {
  char buf[16];
  snprintf(buf, sizeof(buf), "M1:%d", m1);
  Serial2.println(buf);
  delay(5);
  snprintf(buf, sizeof(buf), "M2:%d", m2);
  Serial2.println(buf);
}

inline void sendPosition(float x, float y) {
  target_x = x;
  target_y = y;
  char buf[16];
  snprintf(buf, sizeof(buf), "X:%.1f", x);
  Serial2.println(buf);
  delay(5);
  snprintf(buf, sizeof(buf), "Y:%.1f", y);
  Serial2.println(buf);
}

inline void setModeManuel() {
  mode_auto = false;
  sendToSTM32("mode manuel");
}

inline void setModeAuto() {
  mode_auto = true;
  sendToSTM32("mode auto");
}

inline void stopMotors() {
  sendToSTM32("stop");
}

inline void resetSTM32(int x, int y, int z) {
  char buf[32];
  snprintf(buf, sizeof(buf), "reset:%d:%d:%d", x, y, z);
  Serial2.println(buf);
}

// ============================================
// RECEPTION DONNEES STM32
// ============================================
inline void parseSTM32Data(String& line) {
  // Format: "X:123.4,Y:567.8,Z:1.57"
  int xIdx = line.indexOf("X:");
  int yIdx = line.indexOf("Y:");
  int zIdx = line.indexOf("Z:");
  if (xIdx >= 0 && yIdx >= 0) {
    current_x = line.substring(xIdx + 2, yIdx).toFloat();
    if (zIdx >= 0) {
      current_y = line.substring(yIdx + 2, zIdx).toFloat();
      current_theta = line.substring(zIdx + 2).toFloat();
    } else {
      current_y = line.substring(yIdx + 2).toFloat();
    }
  }
}

inline void receiveFromSTM32() {
  static String rxBuffer = "";
  while (Serial2.available()) {
    char c = Serial2.read();
    if (debugMode) {
      Serial.write(c); // Afficher les logs seulement si debug est désactivé
    }
    if (c == '\n') {
      parseSTM32Data(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
    }
  }
}

// ============================================
// UTILITAIRES NAVIGATION
// ============================================
inline bool checkPosition(float tolerance = 50.0f) {
  return (abs(current_x - target_x) < tolerance && abs(current_y - target_y) < tolerance);
}

#endif
