// Code pour l'ESP32
#include <HardwareSerial.h>

// Définition des pins UART pour l'ESP32 (à adapter selon votre carte)
// Sur beaucoup d'ESP32 WROOM, Serial2 est sur 16(RX) et 17(TX)
#define RXD2 16
#define TXD2 17

// *** MODE DE FONCTIONNEMENT ***
// true = Mode asservissement en position (X, Y, Z, rotation)
// false = Mode vitesse moteurs (M1, M2)
bool MODE_POSITION = false;

void setup() {
  // Serial pour le debug sur l'ordinateur
  Serial.begin(115200);
  
  // Serial2 pour parler au STM32
  // Format: Serial2.begin(BaudRate, Protocol, RX_Pin, TX_Pin);
  // ASSUREZ-VOUS QUE LE BAUDRATE (115200) EST LE MÊME QUE SUR LE STM32
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("Démarrage du test UART vers STM32...");
  Serial.println(MODE_POSITION ? "Mode: ASSERVISSEMENT POSITION" : "Mode: VITESSE MOTEURS");
}

void loop() {
  // Variables statiques pour gérer le temps sans bloquer la boucle
  static unsigned long previousMillis = 0;
  static bool toggle = false;
  const long interval = 2000;

  // 1. Lecture des données reçues de Serial2 (STM32) et renvoi vers Serial (PC)
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }

  // 2. Envoi périodique non bloquant (toutes les 2 secondes)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    if (MODE_POSITION) {
      // *** MODE ASSERVISSEMENT EN POSITION ***
      if (!toggle) {
        Serial2.print("on\n");
        delay(10);
        Serial2.print("X:100\n");
        delay(10);
        Serial2.print("Y:200\n");
        delay(10);
        Serial2.print("Z:45\n"); // Rotation en degrés

      } else {
        Serial2.print("off\n");
        delay(10);
        Serial2.print("X:300\n");
        delay(10);
        Serial2.print("Y:150\n");
        delay(10);
        Serial2.print("Z:270\n"); // Rotation en degrés
      }
    } else {
      // *** MODE VITESSE MOTEURS ***
      if (!toggle) {
        Serial2.print("on\n");
        delay(10);
        Serial2.print("M1:500\n");
        delay(10);
        Serial2.print("M2:0\n");
      } else {
        Serial2.print("off\n");
        delay(10);
        Serial2.print("M1:0\n");
        delay(10);
        Serial2.print("M2:500\n");
      }
    }
    toggle = !toggle;
  }
}