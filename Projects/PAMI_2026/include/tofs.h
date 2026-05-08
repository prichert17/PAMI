#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // Installer "SparkFun VL53L5CX Arduino Library"
#include "config.h" 
#include <vector>


void setup_tof(); 
void loop_tof();
void calcule_points_tof();  // Convertit distances_tof en coordonnées obstacles

// Tableau des distances TOF : [capteur][zone]
// Capteurs : [0]=Centre, [1]=Gauche, [2]=Droite
// Zones : 8 zones de la ligne du milieu de la matrice 8x8
extern float distances_tof[3][8];

// Coordonnées des obstacles détectés dans le repère du robot (mm)
// Ordre : [0]=Centre, [1]=Gauche, [2]=Droite / [zone 0-7]
extern float obstacleX[3][8];
extern float obstacleY[3][8];

// Structure pour représenter un obstacle détecté
struct Obstacle {
    float x;        // Position X dans le repère robot (mm)
    float y;        // Position Y dans le repère robot (mm)
    float distance; // Distance brute du TOF (mm)
    float angle;    // Angle absolu du TOF (degrés, 0°=devant, +90°=gauche)
    uint8_t sensor; // Capteur source : 0=Centre, 1=Gauche, 2=Droite
    uint8_t zone;   // Zone détectée (0-7, colonne de la ligne du milieu)
};

// Liste des obstacles détectés ce cycle (se met à jour chaque loop_tof)
extern std::vector<Obstacle> detectedObstacles;

struct Coordinate {
    int16_t x;
    int16_t y;
  };
  

class Terrain {
    public:
      float distance;
      std::vector<Coordinate> coordinates;  // Tableau dynamique de coordonnées
      Terrain();
      void actualizar();
      void impresion();
      void markLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2);  // Ajouter des coordonnées au tableau
      bool isWithinDistance(float x1, float y1, float x2, float y2, float maxDistance);
      void removeDuplicates(); // Nouvelle fonction pour supprimer les doublons
      void jaune(); //si le terrain est jaune, on créer l'estrade du bon côté
      void bleu();  //si le terrain est bleu, on créer l'estrade du bon côté
      bool estCheminLibre(int local_targetX, int local_targetY);  //
      bool estLibre(int16_t x, int16_t y);
      bool arret_urgence();
      void calcul_new_target();
      int getNumberOfCoordinates(); // Déclaration de la fonction
  };