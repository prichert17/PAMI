/*
 * Test 3x VL53L5CX / VL53L7CX sur ESP32
 * Bibliothèque requise : "VL53L5CX" by STMicroelectronics
 * * Pinout de test :
 * SDA: 21, SCL: 22
 * LPN Tof 1: D23
 * LPN Tof 2: D18
 * LPN Tof 3: D4 
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l5cx_class.h>

// --- Configuration des Pins ---
// LPN (Low Power Enable) = XSHUT
#define LPN_1 23
#define LPN_2 18
#define LPN_3 4

// --- Configuration I2C ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- Instanciation des capteurs ---
// On passe le pointeur Wire et la pin LPN au constructeur
VL53L5CX sensor1(&Wire, LPN_1);
VL53L5CX sensor2(&Wire, LPN_2);
VL53L5CX sensor3(&Wire, LPN_3);

void setup() {
  // 1. Init Série
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n--- Démarrage Test 3x VL53L5CX ---");

  // 2. Init I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz pour accélérer l'upload du firmware

  // 3. Séquence d'allumage et changement d'adresse
  // Pour éviter les conflits, on les éteint tous, puis on les allume un par un
  
  Serial.println("Reset des capteurs...");
  pinMode(LPN_1, OUTPUT);
  pinMode(LPN_2, OUTPUT);
  pinMode(LPN_3, OUTPUT);
  digitalWrite(LPN_1, LOW);
  digitalWrite(LPN_2, LOW);
  digitalWrite(LPN_3, LOW);
  delay(100);

  // --- CAPTEUR 1 ---
  Serial.println("Init Capteur 1 (LPN 23)... (Patienter ~5sec)");
  digitalWrite(LPN_1, HIGH); 
  delay(100);
  if (sensor1.begin()) {
    // Adresse par défaut 0x52 (8-bit) ou 0x29 (7-bit)
    // On change l'adresse vers 0x54 (Attention format 8-bit souvent attendu par cette lib)
    sensor1.vl53l5cx_set_i2c_address(0x54);
    Serial.println("Capteur 1 OK -> Adresse 0x54");
  } else {
    Serial.println("Erreur Init Capteur 1 !");
  }

  // --- CAPTEUR 2 ---
  Serial.println("Init Capteur 2 (LPN 18)... (Patienter ~5sec)");
  digitalWrite(LPN_2, HIGH);
  delay(100);
  if (sensor2.begin()) {
    sensor2.vl53l5cx_set_i2c_address(0x56);
    Serial.println("Capteur 2 OK -> Adresse 0x56");
  } else {
    Serial.println("Erreur Init Capteur 2 !");
  }

  // --- CAPTEUR 3 ---
  Serial.println("Init Capteur 3 (LPN 4)... (Patienter ~5sec)");
  digitalWrite(LPN_3, HIGH);
  delay(100);
  if (sensor3.begin()) {
    sensor3.vl53l5cx_set_i2c_address(0x58);
    Serial.println("Capteur 3 OK -> Adresse 0x58");
  } else {
    Serial.println("Erreur Init Capteur 3 !");
  }

  // 4. Configuration des capteurs (Résolution et Fréquence)
  // Résolutions dispos : VL53L5CX_RESOLUTION_4X4 ou VL53L5CX_RESOLUTION_8X8
  // Fréquence max dépend de la résolution (60Hz en 4x4, 15Hz en 8x8)
  
  Serial.println("Démarrage du Ranging...");
  
  sensor1.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_4X4);
  sensor1.vl53l5cx_set_ranging_frequency_hz(10);
  sensor1.vl53l5cx_start_ranging();

  sensor2.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_4X4);
  sensor2.vl53l5cx_set_ranging_frequency_hz(10);
  sensor2.vl53l5cx_start_ranging();

  sensor3.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_4X4);
  sensor3.vl53l5cx_set_ranging_frequency_hz(10);
  sensor3.vl53l5cx_start_ranging();
}

void loop() {
  VL53L5CX_ResultsData data1, data2, data3;
  uint8_t status1, status2, status3;

  // Lecture non-bloquante
  // On vérifie si des données sont prêtes
  
  // --- Lecture TOF 1 ---
  sensor1.vl53l5cx_check_data_ready(&status1);
  if (status1) {
    sensor1.vl53l5cx_get_ranging_data(&data1);
    // Affichage simple de la distance centrale (Zone 5 sur 16)
    // distance_mm est une matrice [zone]
    Serial.print("T1: ");
    Serial.print(data1.distance_mm[0]); // Zone 0 (coin haut gauche)
    Serial.print("mm | ");
  }

  // --- Lecture TOF 2 ---
  sensor2.vl53l5cx_check_data_ready(&status2);
  if (status2) {
    sensor2.vl53l5cx_get_ranging_data(&data2);
    Serial.print("T2: ");
    Serial.print(data2.distance_mm[0]); 
    Serial.print("mm | ");
  }

  // --- Lecture TOF 3 ---
  sensor3.vl53l5cx_check_data_ready(&status3);
  if (status3) {
    sensor3.vl53l5cx_get_ranging_data(&data3);
    Serial.print("T3: ");
    Serial.print(data3.distance_mm[0]);
    Serial.println("mm");
  }
  
  // Petit delay pour ne pas spammer si aucun capteur ne répond
  delay(5);
}