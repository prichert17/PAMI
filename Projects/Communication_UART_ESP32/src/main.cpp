// Code pour l'ESP32 - Contrôle robot PAMI
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
bool mode_auto = false;  // false = MANUEL, true = AUTO

// Consignes moteurs (mode manuel)
int16_t motor1_cmd = 0;
int16_t motor2_cmd = 0;

// Consignes position (mode auto)
float target_x = 0.0f;
float target_y = 0.0f;

// ============================================
// FONCTIONS D'ENVOI UART (rapide, sans delay)
// ============================================
inline void sendToSTM32(const char* cmd) {
  Serial2.println(cmd);
}

void sendMotor1(int16_t val) {
  char buf[16];
  snprintf(buf, sizeof(buf), "M1:%d", val);
  sendToSTM32(buf);
}

void sendMotor2(int16_t val) {
  char buf[16];
  snprintf(buf, sizeof(buf), "M2:%d", val);
  sendToSTM32(buf);
}

void sendMotors(int16_t m1, int16_t m2) {
  sendMotor1(m1);
  delay(5);
  sendMotor2(m2);
}

void sendPositionX(float x) {
  char buf[16];
  snprintf(buf, sizeof(buf), "X:%.1f", x);
  sendToSTM32(buf);
}

void sendPositionY(float y) {
  char buf[16];
  snprintf(buf, sizeof(buf), "Y:%.1f", y);
  sendToSTM32(buf);
}

void sendPosition(float x, float y) {
  sendPositionX(x);
  sendPositionY(y);
}

void setModeManuel() {
  mode_auto = false;
  sendToSTM32("mode manuel");
  Serial.println(">> Mode MANUEL");
}

void setModeAuto() {
  mode_auto = true;
  sendToSTM32("mode auto");
  Serial.println(">> Mode AUTO");
}

void stopMotors() {
  motor1_cmd = 0;
  motor2_cmd = 0;
  sendToSTM32("stop");
  Serial.println(">> STOP");
}

void setLED(bool on) {
  sendToSTM32(on ? "on" : "off");
  Serial.printf(">> LED %s\n", on ? "ON" : "OFF");
}


// ============================================
// RECEPTION DONNEES STM32
// ============================================
void receiveFromSTM32() {
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("\n=== PAMI ESP32 Control ===");
  Serial.println("mode auto   : Mode automatique");
  Serial.println("mode manuel : Mode manuel");
  Serial.println("--- Mode AUTO ---");
  Serial.println("  X:xxx Y:xxx XY:xxx,yyy");
  Serial.println("--- Mode MANUEL ---");
  Serial.println("  M1:xxx M2:xxx M:xxx,yyy");
  Serial.println("  on / off / stop");
  Serial.println("==========================\n");
  
  // Démarrer en mode manuel
  delay(100);
  setModeManuel();
  //setModeAuto();
  delay(100);
  
  // Test rapide moteurs
  sendMotors(200, 200);   // Avant
  delay(2000);
  sendMotors(0, 0);       // Stop
  delay(1000);
  sendMotors(-200, -200); // Arrière
  delay(2000);
  stopMotors();           // Fin
}

// ============================================
// LOOP PRINCIPAL (le plus rapide possible)
// ============================================
void loop() {
  // Réception données STM32 -> PC
  receiveFromSTM32();
}