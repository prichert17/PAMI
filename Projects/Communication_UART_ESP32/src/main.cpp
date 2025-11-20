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
  Serial.println("Envoi de la commande: on");
  
  // Envoi des caractères 'o' puis 'n'
  Serial2.print("on");
  
  // Attendre 2 secondes
  delay(2000);
  
  // Optionnel : Envoyer autre chose pour voir si ça ne déclenche pas
  Serial2.print("off");
  delay(2000);
}