#include "tofs.h"
#include "types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Définition des pins LPN (XSHUT) et des objets capteurs
// Ordre : Centre (L5CX), Gauche (L7CX), Droite (L7CX)
const int lpnPins[3] = {PIN_TOF2_LPN, PIN_TOF3_LPN, PIN_TOF1_LPN}; 
SparkFun_VL53L5CX sensors[3]; 

// Nouvelles adresses I2C cibles (Le défaut est 0x29)
const byte targetAddresses[3] = {0x30, 0x31, 0x32}; 

// Variables pour stocker les résultats
VL53L5CX_ResultsData measurementData; 

// Flags indiquant quels capteurs sont actifs
bool sensorActive[3] = {false, false, false};

/*Tableaux pour stocker les distances des tofs*/
// Ordre des capteurs : [0]=Centre, [1]=Gauche, [2]=Droite
// On stocke 16 zones (2 lignes centrales de la matrice 8x8)
float distances_tof[3][8]; 

void setup_tof() {
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("\n--- Initialisation tofs ---");

  // Augmenter la vitesse I2C est crucial pour 3 capteurs matriciels
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000); // 400kHz I2C

  // 1. Reset Total : On éteint tous les capteurs
  Serial.println("Reset des capteurs (LPN LOW)...");
  for (int i = 0; i < 3; i++) {
    pinMode(lpnPins[i], OUTPUT);
    digitalWrite(lpnPins[i], LOW); 
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  // 2. Initialisation Séquentielle (Daisy Chain)
  for (int i = 0; i < 3; i++) {
    Serial.printf("Démarrage Capteur %d (Pin %d)... ", i + 1, lpnPins[i]);
    
    // a. Allumer le capteur courant
    digitalWrite(lpnPins[i], HIGH);
    vTaskDelay(pdMS_TO_TICKS(200)); // Délai pour boot hardware du capteur
    
    // b. Essayer d'initialiser à l'adresse par défaut 0x29
    // Note : begin() charge le firmware, cela prend ~1-2 secondes par capteur
    bool initSuccess = false;
    if (sensors[i].begin() == true) {
      // Adresse par défaut OK, changer l'adresse
      if (sensors[i].setAddress(targetAddresses[i]) == true) {
        Serial.printf("OK -> Adresse changée en 0x%02X\n", targetAddresses[i]);
        initSuccess = true;
      } else {
        Serial.println(F("Echec changement adresse!"));
        initSuccess = false;
      }
    } else {
      // Capteur pas détecté à 0x29 : essayer directement à l'adresse cible
      // (cas de reset sans réinitialisaton, les LPN n'ont pas de pull-up)
      Serial.print(F("Pas de réponse à 0x29, essai à 0x"));
      Serial.println(targetAddresses[i], HEX);
      
      // Essayer d'initialiser directement à l'adresse cible
      if (sensors[i].begin(targetAddresses[i]) == true) {
        Serial.printf("OK -> Capteur trouvé à 0x%02X (déjà initialisé précédemment)\n", targetAddresses[i]);
        initSuccess = true;
      } else {
        Serial.println(F("Echec! (Pas de réponse à 0x29 ni à l'adresse cible)"));
        initSuccess = false;
      }
    }
    
    if (initSuccess) {
      // d. Configuration (Résolution 4x4 ou 8x8, Fréquence)
      sensors[i].setResolution(8*8); // 8x8 pour une meilleure résolution
      sensors[i].setRangingFrequency(15); // 15Hz
      sensors[i].startRanging();
      sensorActive[i] = true;
    } else {
      sensorActive[i] = false;
    }
  }
  
  int activeCount = 0;
  for (int i = 0; i < 3; i++) if (sensorActive[i]) activeCount++;
  Serial.printf("\n--- %d/3 capteurs initialisés et rangent ---\n", activeCount);
  
  if (activeCount < 3) {
      Serial.println("ERREUR : Tous les capteurs TOF n'ont pas pu être initialisés.");
      state = STATE_ERROR;
  }
}

void loop_tof() {
  // On boucle sur chaque capteur
  for (int i = 0; i < 3; i++) {
    // Ignorer les capteurs non initialisés
    if (!sensorActive[i]) continue;
    // Vérifie si des données sont prêtes
    if (sensors[i].isDataReady()) {
      if (sensors[i].getRangingData(&measurementData)) {
        
        // Pour ce test minimal, on cherche la distance la plus courte vue par le capteur
        int minDistance = 4000;
        int validZones = 0;

        // Parcourir les 16 zones centrales (lignes 4 et 5 de la matrice 8x8)
        for(int j = 32; j < 40; j++){
            // Statut 5 ou 9 = Mesure valide
            if(measurementData.target_status[j] == 5 || measurementData.target_status[j] == 9){
              distances_tof[i][j - 32] = measurementData.distance_mm[j]; 

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
  vTaskDelay(pdMS_TO_TICKS(50)); // Yield au scheduler RTOS
}

// Position des coordonnées robot (mises à jour depuis pami_com.h)
extern float current_x, current_y, current_theta;

// Offsets des TOF par rapport au centre du robot (en mm)
// [capteur][0]=offset_avant, [capteur][1]=offset_lateral (+ = gauche, - = droite)
const float tofOffsets[3][2] = {
  {20.0f,   0.0f},   // Centre : 2cm devant
  {15.0f,  20.0f},   // Gauche : 1.5cm devant, 2cm à gauche
  {15.0f, -20.0f}    // Droite : 1.5cm devant, 2cm à droite
};

// Angle des capteurs par rapport à l'avant du robot (en radians)
// Centre=0°, Gauche=+55.5°, Droite=-55.5°
const float sensorAngles[3] = {0.0f, 0.9687f, -0.9687f}; // 55.5° = 55.5*PI/180

// FOV de chaque capteur (en radians) : Centre=45°, Gauche/Droite=60°
const float sensorFOV[3] = {0.7854f, 1.0472f, 1.0472f}; // 45°, 60°, 60°

// Stockage des obstacles détectés [capteur][zone]
float obstacleX[3][8];
float obstacleY[3][8];

void calcule_points_tof() {
  float theta = current_theta;
  float cosTheta = cos(theta);
  float sinTheta = sin(theta);
  
  for (int sensor = 0; sensor < 3; sensor++) {
    // Position du capteur dans le référentiel absolu
    float tofX = current_x + tofOffsets[sensor][0] * cosTheta - tofOffsets[sensor][1] * sinTheta;
    float tofY = current_y + tofOffsets[sensor][0] * sinTheta + tofOffsets[sensor][1] * cosTheta;
    
    // Calcul des angles de zone selon le FOV du capteur
    float zoneStep = sensorFOV[sensor] / 8.0f;
    float zoneStart = -sensorFOV[sensor] / 2.0f + zoneStep / 2.0f;
    
    for (int zone = 0; zone < 8; zone++) {
      float dist = distances_tof[sensor][zone];
      if (dist > 0) {
        // Angle total = robot + capteur + zone
        float zoneAngle = zoneStart + zone * zoneStep;
        float totalAngle = theta + sensorAngles[sensor] + zoneAngle;
        
        // Coordonnées de l'obstacle
        obstacleX[sensor][zone] = tofX + dist * cos(totalAngle);
        obstacleY[sensor][zone] = tofY + dist * sin(totalAngle);
      } else {
        obstacleX[sensor][zone] = 0;
        obstacleY[sensor][zone] = 0;
      }
    }
  }
}