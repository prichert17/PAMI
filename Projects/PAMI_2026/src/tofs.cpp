#include "tofs.h"
#include "types.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Définition des pins LPN (XSHUT) et des objets capteurs
// Ordre : [0]=TOF1(Droite), [1]=TOF2(Gauche), [2]=TOF3(Centre)
const int lpnPins[3] = {PIN_TOF1_LPN, PIN_TOF2_LPN, PIN_TOF3_LPN}; 
SparkFun_VL53L5CX sensors[3]; 

// Nouvelles adresses I2C cibles (Le défaut est 0x29)
const byte targetAddresses[3] = {0x30, 0x31, 0x32}; 

// Variables pour stocker les résultats
VL53L5CX_ResultsData measurementData; 

// Flags indiquant quels capteurs sont actifs
bool sensorActive[3] = {false, false, false};

/*Tableaux pour stocker les distances des tofs*/
// Ordre des capteurs : [0]=Centre, [1]=Gauche, [2]=Droite
// On stocke 3 lignes (Haut=0, Milieu=1, Bas=2) de 8 zones
float distances_tof[3][3][8]; 

// Coordonnées (x, y) des obstacles calculées (Ligne du milieu uniquement)
float obstacleX[3][8];
float obstacleY[3][8];

// Liste des obstacles détectés ce cycle
std::vector<Obstacle> detectedObstacles;

// Prototypes des fonctions helper
float getTOFZoneAngle(uint8_t sensor, uint8_t zone);
void distanceAngleToCoord(float distance, float angle, float* outX, float* outY);

// Position et orientation du robot en temps réel (mises à jour depuis pami_com.h)
extern float current_x, current_y, current_theta;

// Offsets des TOF par rapport au centre du robot (en mm)
// Ordre : [0]=TOF1(Droite), [1]=TOF2(Gauche), [2]=TOF3(Centre)
// [capteur][0]=offset_avant, [capteur][1]=offset_lateral (+ = gauche, - = droite)
const float tofOffsets[3][2] = {
  {15.0f, -20.0f},   // TOF1 (Droite) : 1.5cm devant, 2cm à droite
  {15.0f,  20.0f},   // TOF2 (Gauche) : 1.5cm devant, 2cm à gauche
  {20.0f,   0.0f}    // TOF3 (Centre) : 2cm devant
}; 

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

/**
 * Calcule l'angle absolu d'une zone pour un capteur donné
 * @param sensor 0=TOF1(Droite), 1=TOF2(Gauche), 2=TOF3(Centre)
 * @param zone   0-7 (colonne dans la ligne du milieu)
 * @return Angle absolu en degrés (0°=devant, +90°=gauche, -90°=droite)
 */
float getTOFZoneAngle(uint8_t sensor, uint8_t zone) {
  float baseAngle, fov;
  
  if (sensor == 0) {
    // TOF1 = Droite : FOV 60°, décalé de +55.5°
    baseAngle = TOF_ANGLE_DROITE;
    fov = TOF_FOV_DROITE;
  } else if (sensor == 1) {
    // TOF2 = Gauche : FOV 60°, décalé de -55.5°
    baseAngle = TOF_ANGLE_GAUCHE;
    fov = TOF_FOV_GAUCHE;
  } else {
    // TOF3 = Centre : FOV 45°, 0°
    baseAngle = TOF_ANGLE_CENTRE;
    fov = TOF_FOV_CENTRE;
  }
  
  // Calculer l'angle de cette zone relative au capteur
  // Zone 0 : droite du FOV (+fov/2)
  // Zone 7 : gauche du FOV (-fov/2)
  float zoneAngle = baseAngle + (fov/2.0f) - (zone / 7.0f) * fov;
  
  return zoneAngle;
}

/**
 * Convertit distance + angle en coordonnées cartésiennes
 * @param distance Distance en mm
 * @param angle    Angle absolu en degrés
 * @param outX     Pointeur pour retourner la coordonnée X (mm)
 * @param outY     Pointeur pour retourner la coordonnée Y (mm)
 */
void distanceAngleToCoord(float distance, float angle, float* outX, float* outY) {
  float angleRad = angle * PI / 180.0f;
  *outX = distance * cos(angleRad);
  *outY = distance * sin(angleRad);
}

void loop_tof() {
  // Réinitialiser la liste des obstacles détectés ce cycle
  detectedObstacles.clear();
  
  // Paramètres du robot pour transformation de repère
  float robotX = current_x;
  float robotY = current_y;
  float robotTheta = current_theta;  // En degrés
  float robotThetaRad = robotTheta * PI / 180.0f;  // Convertir en radians
  float cosTheta = cos(robotThetaRad);
  float sinTheta = sin(robotThetaRad);
  
  // On boucle sur chaque capteur
  for (int i = 0; i < 3; i++) {
    // Ignorer les capteurs non initialisés
    if (!sensorActive[i]) continue;
    
    // Vérifie si des données sont prêtes
    if (sensors[i].isDataReady()) {
      // SEULEMENT si on a de nouvelles données, réinitialiser les zones
      for (int row = 0; row < 3; row++) {
        for (int z = 0; z < 8; z++) {
          distances_tof[i][row][z] = 0;
        }
      }
      for (int z = 0; z < 8; z++) {
        obstacleX[i][z] = 0;
        obstacleY[i][z] = 0;
      }
      
      if (sensors[i].getRangingData(&measurementData)) {
        
        // Calculer la position du capteur dans le repère absolu
        float tofX = robotX + tofOffsets[i][0] * cosTheta - tofOffsets[i][1] * sinTheta;
        float tofY = robotY + tofOffsets[i][0] * sinTheta + tofOffsets[i][1] * cosTheta;

        // Lire les 3 lignes nécessaires du mode 8x8 (haut, milieu, bas)
        int lineIndices[3] = {56, 24, 0}; // Haut = 56, Milieu = 24, Bas = 0
        
        for(int row = 0; row < 3; row++) {
          for(int zone = 0; zone < 8; zone++){
              int j = lineIndices[row] + zone;
              if(measurementData.target_status[j] == 5 || measurementData.target_status[j] == 9){
                float dist = measurementData.distance_mm[j];
                if(dist > 0 && dist < 500) {
                  distances_tof[i][row][zone] = dist;
                    
                  // Uniquement calculer l'obstacle si c'est la ligne du milieu (row == 1)
                  if(row == 1) {
                    // Calculer l'angle absolu de cette zone dans le repère terrain
                    float zoneAngle = getTOFZoneAngle(i, zone);  // Angle relatif au robot (en degrés)
                    float obstacleAngleDeg = robotTheta + zoneAngle;  // Angle absolu en degrés
                    float obstacleAngleRad = obstacleAngleDeg * PI / 180.0f;  // Convertir en radians
                      
                    // Position de l'obstacle en repère absolu
                    float absX = tofX + dist * cos(obstacleAngleRad);
                    float absY = tofY + dist * sin(obstacleAngleRad);
                      
                    obstacleX[i][zone] = absX;
                    obstacleY[i][zone] = absY;
                      
                    // Ajouter à la liste des obstacles détectés
                    Obstacle obs;
                    obs.x = absX;
                    obs.y = absY;
                    obs.distance = dist;
                    obs.angle = zoneAngle;
                    obs.sensor = i;
                    obs.zone = zone;
                    detectedObstacles.push_back(obs);
                  }
                }
              }
          }
        }

        // Afficher les coordonnées des obstacles détectés par ce capteur (repère absolu)
        Serial.print("TOF"); Serial.print(i+1); Serial.print(": ");
        for(int zone = 0; zone < 8; zone++) {
          if(distances_tof[i][1][zone] > 0 && distances_tof[i][1][zone] < 500) {
            Serial.print("[Z"); Serial.print(zone); Serial.print(": ");
            Serial.print((int)obstacleX[i][zone]); Serial.print("mm,");
            Serial.print((int)obstacleY[i][zone]); Serial.print("mm] ");
          }
        }
        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(10)); // Petit délai pour port série
      }
    }
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Yield au scheduler RTOS
}

// Angle des capteurs par rapport à l'avant du robot (en radians)
// Centre=0°, Gauche=+55.5°, Droite=-55.5°
const float sensorAngles[3] = {0.0f, 0.9687f, -0.9687f}; // 55.5° = 55.5*PI/180

// FOV de chaque capteur (en radians) : Centre=45°, Gauche/Droite=60°
const float sensorFOV[3] = {0.7854f, 1.0472f, 1.0472f}; // 45°, 60°, 60°

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
      // Uniquement se baser sur la ligne du milieu !
      float dist = distances_tof[sensor][1][zone];
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