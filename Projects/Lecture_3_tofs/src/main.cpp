#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // Installer "SparkFun VL53L5CX Arduino Library"

// --- Configuration Pinout ESP32 ---
#define SDA_PIN 21
#define SCL_PIN 22

// Définition des pins LPN (XSHUT) et des objets capteurs
// Ordre : Centre (L5CX), Gauche (L7CX), Droite (L7CX)
const int lpnPins[3] = {23, 18, 4}; 
SparkFun_VL53L5CX sensors[3]; 

// Nouvelles adresses I2C cibles (Le défaut est 0x29)
const byte targetAddresses[3] = {0x30, 0x31, 0x32}; 

// Variables pour stocker les résultats
VL53L5CX_ResultsData measurementData; 

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Initialisation Système de Perception ---");

  // Augmenter la vitesse I2C est crucial pour 3 capteurs matriciels
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C

  // 1. Reset Total : On éteint tous les capteurs
  Serial.println("Reset des capteurs (LPN LOW)...");
  for (int i = 0; i < 3; i++) {
    pinMode(lpnPins[i], OUTPUT);
    digitalWrite(lpnPins[i], LOW); 
  }
  delay(100);

  // 2. Initialisation Séquentielle (Daisy Chain)
  for (int i = 0; i < 3; i++) {
    Serial.printf("Démarrage Capteur %d (Pin %d)... ", i + 1, lpnPins[i]);
    
    // a. Allumer le capteur courant
    digitalWrite(lpnPins[i], HIGH);
    delay(50); // Petit délai pour le boot hardware
    
    // b. Initialiser le capteur à l'adresse par défaut 0x29
    // Note : begin() charge le firmware, cela prend ~1-2 secondes par capteur
    if (sensors[i].begin() == false) {
      Serial.println(F("Echec! (Pas de réponse à 0x29 ou erreur firmware)"));
      while (1); // On bloque ici en cas d'erreur critique
    }
    
    // c. Changer l'adresse I2C
    if (sensors[i].setAddress(targetAddresses[i]) == false) {
      Serial.println(F("Echec changement adresse!"));
      while (1);
    }
    
    Serial.printf("OK -> Adresse changée en 0x%02X\n", targetAddresses[i]);

    // d. Configuration (Résolution 4x4 ou 8x8, Fréquence)
    sensors[i].setResolution(4*4); // 4x4 est plus rapide pour commencer
    sensors[i].setRangingFrequency(15); // 15Hz
    sensors[i].startRanging();
  }
  
  Serial.println("\n--- Tous les capteurs sont prêts et rangent ---");
}

void loop() {
  // On boucle sur chaque capteur
  for (int i = 0; i < 3; i++) {
    // Vérifie si des données sont prêtes
    if (sensors[i].isDataReady()) {
      if (sensors[i].getRangingData(&measurementData)) {
        
        // Pour ce test minimal, on cherche la distance la plus courte vue par le capteur
        int minDistance = 4000;
        int validZones = 0;

        // Parcourir les 16 zones (4x4)
        for(int j = 0; j < 16; j++){
            // Statut 5 ou 9 = Mesure valide
            if(measurementData.target_status[j] == 5 || measurementData.target_status[j] == 9){
                int dist = measurementData.distance_mm[j];
                if(dist < minDistance && dist > 0) {
                    minDistance = dist;
                }
                validZones++;
            }
        }

        // Affichage compact pour le débogage
        Serial.print("C"); Serial.print(i+1); 
        Serial.print(":"); 
        if(validZones > 0) Serial.print(minDistance); else Serial.print("---");
        Serial.print("mm\t");
      }
    }
  }
  Serial.println(); // Nouvelle ligne après avoir checké les 3
  delay(50); // Petit délai pour ne pas spammer le port série (à retirer en prod)
}