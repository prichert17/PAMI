/* Version 0.1*/
#include "TOF_2.h"

#define H 200 //cm
#define L 300 //cm
#define R  0 //Il n'y pas d'objet (Rien)
#define O  1 //Il y a un objet
#define S  1 // Estrade

#define ECHO1 4
#define TRIGG1 5
#define ECHO2 15
#define TRIGG2 2

extern BluetoothSerial SerialBT;
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 bytes of RAM

 
//variables pour le timer de l'ultrason
unsigned long previousTime1 = 0;
unsigned long previousTime2 = 0;
const unsigned long interval1 = 100;  // Intervalle de mesure en ms (capteur 1)
const unsigned long interval2 = 120;  // Intervalle légèrement différent pour éviter les interférences

// Variables pour les capteurs ultrasons
volatile long duration1;
volatile long duration2;
float distance1;
float distance2;
float distance_tof1;
float distance_tof2;
float distance_tof3;
float distance_tof4;

float tofX, tofY, tofTheta;
float tofX_1, tofY_1;
float tofX_2, tofY_2;
float tofX_3, tofY_3;
float tofX_4, tofY_4;

extern float robotX; 
extern float robotY; 
extern float robotTheta;

extern float targetX; 
extern float targetY; 

extern float duree_pid;

unsigned long lastTime2 = 0;     // Temps du dernier calcul

extern bool couleur;

extern int numCoordinates;

bool presence_obstacle = false;

Terrain ter;


Terrain::Terrain() {
  // On n'a pas de constructeur 
}

bool Terrain::arret_urgence() {
  // Fonction pour choisir la couleur du terrain
  distance_tof1 = measurementData.distance_mm[8]/10; // distance en cm
  distance_tof2 = measurementData.distance_mm[9]/10; // distance en cm
  distance_tof3 = measurementData.distance_mm[10]/10; // distance en cm
  distance_tof4 = measurementData.distance_mm[11]/10; // distance en cm
  if ((distance1 < 7) && (distance1 != 0)) {
    return true;
  }else if ((distance2 < 7) && (distance2 != 0)) {
    return true;
  }else if((distance_tof1<7) && (distance_tof1 != 0)){
    return true;
  }else if((distance_tof2<7) && (distance_tof2 != 0)){
    return true;
    
  }else if((distance_tof3<7) && (distance_tof3 != 0)){
    return true;
    
  }else if((distance_tof4<7) && (distance_tof4 != 0)){
    return true;
    
  }
  return false;

}

bool Terrain::isWithinDistance(float x1, float y1, float x2, float y2, float maxDistance) {
  // Vérifie si la distance entre deux points est inférieure ou égale à maxDistance
  float distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  return distance <= maxDistance;
}

void Terrain::markLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  // Fonction de tracé de ligne de Bresenham et ajout des coordonnées dans le tableau
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int sx = x1 < x2 ? 1 : -1;
  int sy = y1 < y2 ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (x1 >= -L && x1 < L && y1 >= -H && y1 < H) {
      // Ajouter les coordonnées dans le tableau
      coordinates.push_back({(int16_t)x1, (int16_t)y1});
    }
    if (x1 == x2 && y1 == y2) break;
    int e2 = err * 2;
    if (e2 > -dy) { err -= dy; x1 += sx; }
    if (e2 < dx) { err += dx; y1 += sy; }
  }
} 
void Terrain::actualizar() {
  // Marquer les lignes entre les points TOF si ils sont à moins de 20 cm de distance
  if (isWithinDistance(tofX_1, tofY_1, tofX_2, tofY_2, 20)) {
    markLine(round(tofX_1), round(tofY_1), round(tofX_2), round(tofY_2));
  }
  if (isWithinDistance(tofX_2, tofY_2, tofX_3, tofY_3, 20)) {
    markLine(round(tofX_2), round(tofY_2), round(tofX_3), round(tofY_3));
  }
  if (isWithinDistance(tofX_3, tofY_3, tofX_4, tofY_4, 20)) {
    markLine(round(tofX_3), round(tofY_3), round(tofX_4), round(tofY_4));
  }
  if (isWithinDistance(tofX_4, tofY_4, tofX_1, tofY_1, 20)) {
    markLine(round(tofX_4), round(tofY_4), round(tofX_1), round(tofY_1));
  }

  // Supprimer les doublons après chaque mise à jour
  removeDuplicates();

}


void Terrain::jaune() {
  // Premier rectangle : 60 à 105 (x), 0 à 20 (y)
  //markLine(60, 0, 105, 0);      // Bas
  markLine(60, 20, 105, 20);    // Haut
  markLine(60, 0, 60, 20);      // Gauche
  //markLine(105, 0, 105, 20);    // Droite

  // Deuxième rectangle : 105 à 195 (x), 0 à 45 (y)
  //markLine(105, 0, 195, 0);    // Bas
  markLine(105, 45, 195, 45);   // Haut
  markLine(105, 0, 105, 45);    // Gauche
  markLine(195, 0, 195, 45);    // Droite
}


void Terrain::bleu() {
  // Premier rectangle : 60 à 105 (x), -20 à 0 (y)
  markLine(60, -20, 105, -20);  // Bas
  //markLine(60, 0, 105, 0);      // Haut
  markLine(60, -20, 60, 0);     // Gauche
  //markLine(105, -20, 105, 0);   // Droite

  // Deuxième rectangle : 105 à 195 (x), -45 à 0 (y)
  markLine(105, -45, 195, -45); // Bas
  //markLine(105, 0, 195, 0);     // Haut
  markLine(105, -45, 105, 0);   // Gauche
  markLine(195, -45, 195, 0);   // Droite
}


void Terrain::removeDuplicates() {
  // Créer un nouveau vecteur pour stocker les coordonnées sans doublon
  std::vector<Coordinate> uniqueCoordinates;

  for (const auto& coord : coordinates) {
    bool exists = false;
    // Vérifier si la coordonnée existe déjà dans uniqueCoordinates
    for (const auto& uniqueCoord : uniqueCoordinates) {
      if (coord.x == uniqueCoord.x && coord.y == uniqueCoord.y) {
        exists = true;
        break;
      }
    }
    // Si la coordonnée n'existe pas déjà, on l'ajoute
    if (!exists) {
      uniqueCoordinates.push_back(coord);
    }
  }

  // Mettre à jour le tableau de coordonnées avec les coordonnées uniques
  coordinates = uniqueCoordinates;
}

void Terrain::impresion() {
  // Afficher les coordonnées stockées
  
  for (size_t i = 0; i < coordinates.size(); i++) {
    Serial.print("X: ");
    Serial.print(coordinates[i].x);
    Serial.print(" Y: ");
    Serial.println(coordinates[i].y);
    SerialBT.print("X: ");
    SerialBT.print(coordinates[i].x);
    SerialBT.print(" Y: ");
    SerialBT.println(coordinates[i].y);
  }
}

//Variable qui sont utilisés pour le set-up
int imageResolution = 0;
int imageWidth = 0; 


void calcule_points_tof(const int16_t p1, const int16_t p2, const int16_t p3, const int16_t p4) {
  tofX = robotX + 10 * cos(robotTheta);
  tofY = robotY + 10 * sin(robotTheta);
  if (p1 > 0){
  tofX_1 = round(tofX + p1 * cos((robotTheta - 22.5 * PI / 180)));    //22,5° en radians, car c'est la moitié de l'angle max des tofs (45°)
  tofY_1 = round(tofY + p1 * sin((robotTheta - 22.5 * PI / 180)));
  }
  if (p2 > 0){
  tofX_2 = round(tofX + p2 * cos((robotTheta - 7.5 * PI / 180)));     // 7,5° en radians car on a deux tofs qui sont à 15° l'un de l'autre
  tofY_2 = round(tofY + p2 * sin((robotTheta - 7.5 * PI / 180)));
  }
  if (p3 > 0){
  tofX_3 = round(tofX + p3 * cos((robotTheta + 7.5 * PI / 180)));
  tofY_3 = round(tofY + p3 * sin((robotTheta + 7.5 * PI / 180)));
  }if (p4 > 0){
  tofX_4 = round(tofX + p4 * cos((robotTheta + 22.5 * PI / 180)));
  tofY_4 = round(tofY + p4 * sin((robotTheta + 22.5 * PI / 180)));
  }
}




// Fonction de mesure générique pour les deux capteurs
float mesureUltrasons(int trigPin, int echoPin) {
  long duration;
  
  // Envoyer une impulsion de 10µs sur la broche TRIG
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Mesurer le temps de l'écho
  duration = pulseIn(echoPin, HIGH);
  
  // Calculer la distance (vitesse du son = 340 m/s)
  // distance = (temps × vitesse du son) / 2
  return duration * 0.034 / 2;
}

bool Terrain::estLibre(int16_t x, int16_t y) {
  for (const auto& coord : coordinates) {
    if (coord.x == x && coord.y == y) {
      return false; // Coordonnée occupée
      Serial.println("Obstacle");
      SerialBT.println("Obstacle");
    }
  }
  return true; // Coordonnée libre
  
}



bool Terrain::estCheminLibre(int local_targetX, int local_targetY) {
  const float pas = 1.0; // Pas de progression en cm pour la vérification
  const float distance = sqrt(pow(local_targetX - robotX, 2) + pow(local_targetY - robotY, 2)) + 7; // Distance totale à vérifier, ajout de 7 cm pour que l'arrêt d'urgence ne s'allume pas
  const int nbSteps = distance / pas;

  // Calcul de la direction unitaire vers la cible
  float dirX = (local_targetX - robotX) / distance;
  float dirY = (local_targetY - robotY) / distance;

  // Vérifie chaque point le long de la ligne
  for (int i = 0; i <= nbSteps; ++i) {
      float posX = robotX + dirX * i * pas;
      float posY = robotY + dirY * i * pas;

      if (!ter.estLibre(round(posX), round(posY))) { // Vérifie si le point est libre
          return false; // Un obstacle a été détecté
      }
  }

  return true; // Tous les points sont libres
}

void Terrain::calcul_new_target(){    // Calcul de la nouvelle cible si il y a un obstacle
  /*
  Serial.print("Ancienne cible - X: ");
  Serial.print(targetX);
  Serial.print(", Y: ");
  Serial.println(targetY);
  */
 
  // Calcul de la distance entre la position actuelle du robot et la cible
  float distance = sqrt(pow(targetX - robotX, 2) + pow(targetY - robotY, 2));
  if (distance < 20) {
    distance = 20; // Assure que la distance est au moins de 20 cm
  }

  // Calcul de l'angle actuel entre le robot et la cible
  float angleToTarget = atan2(targetY - robotY, targetX - robotX);

  // vérifie si le chemin est libre en testant un angle différent tant que le chemin n'est pas libre
  
  
  float newAngle = angleToTarget;
  while (!ter.estCheminLibre(robotX + distance * cos(newAngle), robotY + distance * sin(newAngle))) {
    if (angleToTarget > 0) { // Si l'angle est positif, on tourne dans le sens antihoraire
    newAngle -= (1 * PI / 180); // Augmente l'angle par incréments de -1°
    }else{
      newAngle += (1 * PI / 180); // Diminue l'angle par incréments de 1°
    }
    //Serial.print("Coordonnée testées: ");
    //Serial.print(robotX + distance * cos(newAngle));
    //Serial.print(", ");
    //Serial.println(robotY + distance * sin(newAngle));
    if (newAngle >= angleToTarget + (180 * PI / 180)) { // Évite une boucle infinie
      Serial.println("Aucun chemin libre trouvé.");
      SerialBT.println("Aucun chemin libre trouvé.");
      return; // Sort de la fonction si aucun chemin n'est trouvé
    }
      
  }/*
  if (angleToTarget > 0){   //comme on ne prend pas en compte la largeur du robot, on rajoute 5° pour que le robot ne se cogne pas
    newAngle += 5;
  }else{
    newAngle -= 5;
  }*/
  //Serial.print("Nouvel angle: ");
  //Serial.println(newAngle * 180.0 / PI); // Affiche l'angle en degrés

  //float newAngle = angleToTarget + (20 * PI / 180);

  // Calcul des nouvelles coordonnées cibles en utilisant la distance et le nouvel angle
  targetX = robotX + distance * cos(newAngle);
  targetY = robotY + distance * sin(newAngle);


  // Affichage des nouvelles coordonnées pour le débogage
  //Serial.print("Nouvelle cible - X: ");
  //Serial.print(targetX);
  //Serial.print(", Y: ");
  //Serial.println(targetY);
  
  
}

int Terrain::getNumberOfCoordinates() {
  return coordinates.size(); // Retourne la taille du vecteur coordinates
}



void tof_setup_spark()
{

  pinMode(TRIGG1, OUTPUT);
  pinMode(TRIGG2, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);

  Serial.begin(115200);
  Serial.println("SparkFun VL53L5CX Imager Example");
  SerialBT.print("SparkFun VL53L5CX Imager Example");

  Wire.begin(); 
  Wire.setClock(1000000);


  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  SerialBT.println("Initializing sensor board. This can take up to 10s. Please wait.");

  long startTime = millis();
  bool startup = myImager.begin();
  long stopTime = millis();

  if (startup == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    SerialBT.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }

  Serial.print("Firmware transfer time: ");
  float timeTaken = (stopTime - startTime) / 1000.0;
  Serial.print(timeTaken, 3);
  Serial.println("s");
  
  //16 zones sur le capteur TOF
  myImager.setResolution(4*4); 
  imageResolution = myImager.getResolution(); 
  imageWidth = sqrt(imageResolution); 
    bool response = myImager.setRangingFrequency(20);
  if (response == true)
  {
    int frequency = myImager.getRangingFrequency();
    if (frequency > 0)
    {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    }
    else
      Serial.println(F("Error recovering ranging frequency."));
      SerialBT.println(F("Error recovering ranging frequency."));
  }
  else
  {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
    SerialBT.println(F("Cannot set ranging frequency requested. Freezing..."));
    while (1) ;
  }
  myImager.startRanging();
  
}

void tof_loop_spark()
{
    
  unsigned long currentTime = millis();

  if (currentTime - lastTime2 >= duree_pid) {
  
  lastTime2 = currentTime;

  // Vérifier si l'intervalle est écoulé pour le capteur 1
  if (currentTime - previousTime1 >= interval1) {
    previousTime1 = currentTime;
    
    // Appeler la fonction de mesure pour le capteur 1
    distance1 = mesureUltrasons(TRIGG1, ECHO1);
    
    // Afficher la distance du capteur 1
    //Serial.print("Distance capteur 1: ");
    //Serial.print(distance1);
    //Serial.println(" cm");
  }
  
  // Vérifier si l'intervalle est écoulé pour le capteur 2
  if (currentTime - previousTime2 >= interval2) {
    previousTime2 = currentTime;
    
    // Appeler la fonction de mesure pour le capteur 2
    distance2 = mesureUltrasons(TRIGG2, ECHO2);
    
    // Afficher la distance du capteur 2
    //Serial.print("Distance capteur 2: ");
    //Serial.print(distance2);
    //Serial.println(" cm");
  }


if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Lecture de la distance data dans une array
    {
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
         
          if(measurementData.distance_mm[x + y]/25 < 10 && (y < 12 && y >= 8 )){
          
            calcule_points_tof(measurementData.distance_mm[8]/10,measurementData.distance_mm[9]/10,measurementData.distance_mm[10]/10,measurementData.distance_mm[11]/10);
          
          
          
          }

        }
        
      }
      
    }

  }
  ter.actualizar();

  presence_obstacle = !ter.estCheminLibre(targetX,targetY);     //regarde toutes les coordonnées entre son point et sa trajectoire cible

  if (presence_obstacle == true){
      //Serial.println("Obstacle");
      //ter.impresion();
      ter.calcul_new_target();
    }
  
  Serial.print("TargetX : ");
  Serial.print(targetX);
  Serial.print(" TargetY : ");
  Serial.println(targetY);

  Serial.print("RobotX : ");
  Serial.print(robotX);
  Serial.print(" RobotY : ");
  Serial.println(robotY);

  SerialBT.print("TargetX : ");
  SerialBT.print(targetX);
  SerialBT.print(" TargetY : ");
  SerialBT.println(targetY);

  SerialBT.print("RobotX : ");
  SerialBT.print(robotX);
  SerialBT.print(" RobotY : ");
  SerialBT.println(robotY);
  
    //Serial.println();
    //Serial.println("Obstacles : ");
    //ter.impresion();
  }
}
