#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

// Pins I²C ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// Adresses des capteurs détectées précédemment
#define ADDR_GAUCHE 0x29  // par défaut
#define ADDR_DROITE 0x54  // second capteur déjà reconfiguré

// Capteurs VL53L7CX
SparkFun_VL53L5CX tofGauche, tofDroite;
VL53L5CX_ResultsData dataGauche, dataDroite;

unsigned long lastTime = 0;
const unsigned long dureeMesure = 200; // 5 Hz (200ms)

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // Fréquence I²C 400kHz

  Serial.println("Initialisation capteurs VL53L7CX...");

  // Initialisation Gauche (0x29)
  if (!tofGauche.begin(ADDR_GAUCHE)) {
    Serial.println("Erreur capteur gauche à l'adresse 0x29.");
    while (1);
  }
  tofGauche.setResolution(8 * 8); // mode 8×8
  tofGauche.setRangingFrequency(5);
  tofGauche.startRanging();
  Serial.println("Capteur Gauche (0x29) OK !");

  // Initialisation Droite (0x56)
  if (!tofDroite.begin(ADDR_DROITE)) {
    Serial.println("Erreur capteur droite à l'adresse 0x56.");
    while (1);
  }
  tofDroite.setResolution(8 * 8); // mode 8×8
  tofDroite.setRangingFrequency(5);
  tofDroite.startRanging();
  Serial.println("Capteur Droite (0x56) OK !");
}

void loop() {
  if (millis() - lastTime >= dureeMesure) {
    lastTime = millis();

    // Lecture Gauche
    if (tofGauche.isDataReady() && tofGauche.getRangingData(&dataGauche)) {
      Serial.print("VL53L7CX Gauche (ligne milieu) : ");
      for (int i = 24; i < 32; i++) {
        Serial.print(dataGauche.distance_mm[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Pas de données Gauche");
    }

    // Lecture Droite
    if (tofDroite.isDataReady() && tofDroite.getRangingData(&dataDroite)) {
      Serial.print("VL53L7CX Droite (ligne milieu) : ");
      for (int i = 24; i < 32; i++) {
        Serial.print(dataDroite.distance_mm[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Pas de données Droite");
    }

    Serial.println();
  }
}
