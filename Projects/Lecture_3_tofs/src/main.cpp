#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#define XSHUT_CENTRE  18
#define XSHUT_GAUCHE  19
#define XSHUT_DROITE  23

#define ADDR_CENTRE  0x29   // 0x52 >> 1
#define ADDR_GAUCHE  0x2A   // 0x54 >> 1
#define ADDR_DROITE  0x2B   // 0x56 >> 1

SparkFun_VL53L5CX tof_centre;
SparkFun_VL53L5CX tof_gauche;
SparkFun_VL53L5CX tof_droite;
VL53L5CX_ResultsData data_centre, data_gauche, data_droite;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(XSHUT_CENTRE, OUTPUT);
  pinMode(XSHUT_GAUCHE, OUTPUT);
  pinMode(XSHUT_DROITE, OUTPUT);

  // Éteins tous les capteurs
  digitalWrite(XSHUT_CENTRE, LOW);
  digitalWrite(XSHUT_GAUCHE, LOW);
  digitalWrite(XSHUT_DROITE, LOW);
  delay(50);
  
  // Démarre Centre, change son adresse
  digitalWrite(XSHUT_CENTRE, HIGH);
  delay(10);
  if (!tof_centre.begin(ADDR_CENTRE, Wire)) {
    Serial.println("Erreur capteur CENTRE !");
    while (1);
  }
  tof_centre.setAddress(ADDR_CENTRE);

  // Démarre Gauche, change son adresse
  digitalWrite(XSHUT_GAUCHE, HIGH);
  delay(10);
  if (!tof_gauche.begin(ADDR_GAUCHE, Wire)) {
    Serial.println("Erreur capteur GAUCHE !");
    while (1);
  }
  tof_gauche.setAddress(ADDR_GAUCHE);

  // Démarre Droite, change son adresse
  digitalWrite(XSHUT_DROITE, HIGH);
  delay(10);
  if (!tof_droite.begin(ADDR_DROITE, Wire)) {
    Serial.println("Erreur capteur DROITE !");
    while (1);
  }
  tof_droite.setAddress(ADDR_DROITE);

  // Paramètres identiques pour tous
  tof_centre.setResolution(16);
  tof_gauche.setResolution(16);
  tof_droite.setResolution(16);

  tof_centre.setRangingFrequency(15); // Hz
  tof_gauche.setRangingFrequency(15);
  tof_droite.setRangingFrequency(15);

  tof_centre.startRanging();
  tof_gauche.startRanging();
  tof_droite.startRanging();

  Serial.println("Init OK.");
}

void loop() {
  // Affichage simple, juste la première valeur de chaque capteur (exemple)
  if (tof_centre.isDataReady())
    if (tof_centre.getRangingData(&data_centre))
      Serial.printf("Centre: %d mm\n", data_centre.distance_mm[8]); // zone centrale

  if (tof_gauche.isDataReady())
    if (tof_gauche.getRangingData(&data_gauche))
      Serial.printf("Gauche: %d mm\n", data_gauche.distance_mm[8]);

  if (tof_droite.isDataReady())
    if (tof_droite.getRangingData(&data_droite))
      Serial.printf("Droite: %d mm\n", data_droite.distance_mm[8]);

  delay(50);
}
