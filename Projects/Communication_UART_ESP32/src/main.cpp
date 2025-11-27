// Code pour l'ESP32
#include <HardwareSerial.h>

// Définition des pins UART pour l'ESP32 (à adapter selon votre carte)
// Sur beaucoup d'ESP32 WROOM, Serial2 est sur 16(RX) et 17(TX)
#define RXD2 16
#define TXD2 17

void setup() {
  // Serial pour le debug sur l'ordinateur
  Serial.begin(115200);
  
  // Serial2 pour parler au STM32
  // Format: Serial2.begin(BaudRate, Protocol, RX_Pin, TX_Pin);
  // ASSUREZ-VOUS QUE LE BAUDRATE (115200) EST LE MÊME QUE SUR LE STM32
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("Démarrage du test UART vers STM32...");
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
    
    if (!toggle) {
      //Serial.println("Envoi de la commande: on");
      Serial2.print("on\n");
      delay(10); // Petit délai pour s'assurer que les commandes sont séparées
      Serial2.print("M1:0\n");
      delay(10); // Petit délai pour s'assurer que les commandes sont séparées
      Serial2.print("M2:0\n");
    } else {
      //Serial.println("Envoi de la commande: off");
      Serial2.print("off\n");
      delay(10); // Petit délai pour s'assurer que les commandes sont séparées
      Serial2.print("M1:0\n");
      delay(10); // Petit délai pour s'assurer que les commandes sont séparées
      Serial2.print("M2:800\n");
    }
    toggle = !toggle;
  }
}