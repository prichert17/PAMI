/* Version 0.1*/
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <vector>
#include "BluetoothSerial.h"


void tof_setup_spark();
void tof_loop_spark();
void calcule_points_tof(const int16_t p1, const int16_t p2, const int16_t p3, const int16_t p4);
float mesureUltrasons(int trigPin, int echoPin);
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