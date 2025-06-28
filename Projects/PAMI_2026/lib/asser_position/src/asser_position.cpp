#include "asser_position.h"

/*Def Pins*/

// Définition des pins pour le moteur 1 gauche
const int motorPin1 = 25;  // Pin du moteur 1 (PWM sens 1)
const int motorPin2 = 26;  // Pin du moteur 1 (PWM sens 2)
const int encoderPinA = 27; // Pin de l'encodeur 1 A
const int encoderPinB = 14; // Pin de l'encodeur 1 B
// Définition des pins pour le moteur 2 droit
const int motorPin3 = 32;  // Pin du moteur 2 (PWM sens 1)
const int motorPin4 = 33;  // Pin du moteur 2 (PWM sens 2)
const int encoderPinC = 35; // Pin de l'encodeur 2 A
const int encoderPinD = 34; // Pin de l'encodeur 2 B
// Définition de la pin du servomoteur
const int servoPin = 18;
// Création d'un objet Servo
Servo myServo;

// Déclaration du timer hardware
hw_timer_t *timer = NULL;
volatile bool timerFlag = false; // Flag pour indiquer qu'une interruption a eu lieu

void IRAM_ATTR onTimer() {
  timerFlag = true; // Définir le flag à true dans l'interruption
}

/*Def variable asservissement*/

// Variables pour la gestion de l'encodeur 1
volatile int pulseCount1 = 0;    // Nombre d'impulsions pour le moteur 1
float speed1 = 0.0;              // Vitesse du moteur 1 en tours par seconde
int directionSpeed1 = 1;         // Direction du moteur 1
// Variables pour la gestion de l'encodeur 2
volatile int pulseCount2 = 0;    // Nombre d'impulsions pour le moteur 2
float speed2 = 0.0;              // Vitesse du moteur 2 en tours par seconde
int directionSpeed2 = 1;         // Direction du moteur 2
// Nombre d'impulsions par tour (selon la fiche technique)
const int pulsesPerRevolution = 1200;  // Valeur mise à jour
// Paramètres pour le PID
unsigned long lastTime1 = 0;     // Temps du dernier calcul
//unsigned long lastTime2 = 0;     // Temps du dernier calcul
//unsigned long lastTime3 = 0;     // Temps du dernier calcul
//unsigned long lastTime4 = 0;     // Temps du dernier calcul

float kp = 0.5, ki = 0.3, kd = -0.2;                              //Coeffs à ajuster
float targetSpeed1 = 100; // Vitesse cible moteur 1
float targetSpeed2 = 100; // Vitesse cible moteur 2
float error1 = 0, error2 = 0;
float prevError1 = 0, prevError2 = 0;
float integral1 = 0, integral2 = 0;
float duree_pid = 100;          //duree du PID en ms (intervalle de temps entre deux calcul)
int cmd1 = 0;     //commande qui sera envoyée au moteur 1 (gauche)
int cmd2 = 0;     //commande qui sera envoyée au moteur 2 (droit)
int prevcmd1 = 0;
int prevcmd2 = 0;


float distance_target = 0;           //distance de déplacement (en X et en Y)
float angle_target = 0/(180/PI);     //angle de déplacement voulu, en radian

//Paramètres PID position
float Idistance_target = 0;       //coeff pour l'intégral
float Iangle_target = 0;
float erreur_distance = 0;          //distance qu'il reste à parcourir
float erreur_angle = 0;

#define K_angle_prop 1      //coeff PID déplacement
#define K_angle_int 0.2
//#define K_angle_prop 1.3     //coeff PID déplacement
//#define K_angle_int 0.4

#define K_deplacement_prop 0.2    //coeff PID angle
#define K_deplacement_int 0.006        //très faible car l'intégrale augmente très vite

int cmdV=0;                  //commande de vitesse qui sera réajustée par la suite (avec la bonne accélération)
float Ez = 0;                //commande d'angle
bool destination_atteinte = false;     //booléen pour savoir si la fonction de déplacement est finie
bool destination_finale_atteinte = false;

// vitesse max
int maxSpeed = 80;      //vitesse max à ne pas dépasser
int minSpeed = 40;
int maxAcc = 5;         //coeff à ajuster selon l'accélération souhaitée

// Déclaration des variables supplémentaires pour la position du robot
float robotX = 3.0;  // Position X du robot en cm
float robotY = 41.0;  // Position Y du robot en cm
float robotTheta = 0.0;  // Orientation du robot en radians
//Position du robot avant déplacement
float robotX_previous = 0.0;  // Position X du robot en cm
float robotY_previous = 0.0;  // Position Y du robot en cm
float robotTheta_previous = 0.0;  // Orientation du robot en radians

//Variable pour la position target du robot
// Déclaration d'un tableau de coordonnées (targetX, targetY)
float coordinates_temp_jaune[][2] = {
  
  {3, 41.0}, // Première coordonnée (targetX, targetY)
  //{70,80},
  {60,41},
  {100,90},
  {200,90},
  
  //{50.0, 18.0},
  //{80.0, 25.0},
};

//même chose mais pour le bleu :
float coordinates_temp_bleu[][2] = {
  {3, -35.0},
  {60,-40},
  {100,-90},
  {200,-90},

};


int i = 0;  // Ajout d'une variable pour gérer le délai
unsigned long lastArrivalTime = 0; // Temps de la dernière arrivée, pour changer de coordonnées cibles

// Taille du tableau
int numCoordinates = sizeof(coordinates_temp_jaune) / sizeof(coordinates_temp_jaune[0]);

//Variable pour la position target du robot
float targetX = 0;  // Position cible X du robot en cm
float targetY = 0;// Position cible Y du robot en cm
//float targetTheta = 0.0;  // Orientation du robot en radians, important ?

float deltaX = 0;
float deltaY = 0;
float targetTheta = 0;
float deltaTheta = 0;


// Diamètre des roues en cm
const float wheelDiameter = 4.30;  
const float wheelCircumference = wheelDiameter * PI;
// Distance entre les roues en cm (les centres des deux roues, cf modélisation 3D)
const float wheelBase = 7.40;

// Variables pour la gestion de l'angle du servo sans delay
int angleServo = 180;
int angleMin = 130;   // Angle minimum
int angleMax = 180;   // Angle maximum
int stepServo = -1;   // Direction du mouvement (-1 pour descendre, +1 pour monter)
unsigned long previousServoMillis = 0;
const int servoInterval = 25;  // Intervalle de mise à jour en millisecondes

extern bool presence_obstacle;

extern bool couleur;  //on dit que true = jaune et false = bleu, variable de test

extern float distance1;
extern float distance2;

extern float distance_tof1;
extern float distance_tof2;
extern float distance_tof3;
extern float distance_tof4;


// Déclaration du timer hardware pour le servo
hw_timer_t *servoTimer = NULL;
portMUX_TYPE servoTimerMux = portMUX_INITIALIZER_UNLOCKED; // Protection pour les variables partagées

float angle_limit(float angle){     //à tester lors du calcul de la targetTheta
  // Fonction pour limiter l'angle entre -PI et PI
  while (angle > PI) {
    angle -= 2 * PI;
  }
  while (angle < -PI) {
    angle += 2 * PI;
  }
  return angle;
}

void asser_position_setup() {
  // Initialisation des pins pour le moteur 1
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Initialisation des pins pour le moteur 2
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(encoderPinC, INPUT);
  pinMode(encoderPinD, INPUT);

  // Attache les interruptions pour les encodeurs
  attachInterrupt(digitalPinToInterrupt(encoderPinA), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinC), countPulse2, RISING);
  myServo.attach(servoPin);

  // Configuration du timer hardware pour une interruption toutes les 100 ms
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 µs par tick), compteur incrémental
  timerAttachInterrupt(timer, &onTimer, true); // Attacher l'interruption au timer
  timerAlarmWrite(timer, 100000, true); // Déclencher l'interruption toutes les 100 ms (100000 µs)
  timerAlarmEnable(timer); // Activer l'alarme du timer
  setupServoInterrupt(); // Initialisation de l'interruption pour le servo

  //setup des premières coordonnées
    if(couleur == true){
      robotX = coordinates_temp_jaune[0][0];
      robotY = coordinates_temp_jaune[0][1];
      targetX = coordinates_temp_jaune[1][0];
      targetY = coordinates_temp_jaune[1][1];
    }else{
      robotX = coordinates_temp_bleu[0][0];
      robotY = coordinates_temp_bleu[0][1];
      targetX = coordinates_temp_bleu[1][0];
      targetY = coordinates_temp_bleu[1][1];
    }

}



// Fonction pour compter les impulsions de l'encodeur 1 et déterminer la direction
void countPulse1() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    directionSpeed1 = -1;  // Sens positif
  } else {
    directionSpeed1 = 1; // Sens négatif
  }
  pulseCount1++;
}

// Fonction pour compter les impulsions de l'encodeur 2 et déterminer la direction
void countPulse2() {
  if (digitalRead(encoderPinC) == digitalRead(encoderPinD)) {
    directionSpeed2 = -1;  // Sens positif
  } else {
    directionSpeed2 = 1; // Sens négatif
  }
  pulseCount2++;
}


  // Fonction pour appliquer les vitesses aux moteurs
void set_vitesse(float cmd1, float cmd2) {
  // Moteur 1
  if (cmd1 > 0) {
    analogWrite(motorPin1, min(abs(cmd1), 255.0f));
    analogWrite(motorPin2, 0);
  } else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, min(abs(cmd1), 255.0f));
  }

  // Moteur 2
  if (cmd2 > 0) {
    analogWrite(motorPin3, min(abs(cmd2), 255.0f));
    analogWrite(motorPin4, 0);
  } else {
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, min(abs(cmd2), 255.0f));
  }
}


void go_to(){
  // Fonction pour déplacer le robot vers une destination, avec les variables globales x_target, y_target et theta_target
  //calcul distance/angle
  deltaX = targetX - robotX;
  deltaY = targetY - robotY;
  targetTheta = atan2(deltaY, deltaX);
  deltaTheta = targetTheta - robotTheta;
  

  distance_target = sqrt(deltaX*deltaX + deltaY*deltaY);           //distance de déplacement (en X et en Y)
  if (distance_target > 2){
    angle_target = deltaTheta;     //si la distance est supérieure à 2, on maj l'angle cible pour déplacer le robot (sinon tourne en rond autour du point à la fin)
  }
  else{
    angle_target = 0;
  }

/*
  if (angle_target > PI/4){
    deplacement_angulaire_horaire();
  }else if(angle_target < -PI/4){
    deplacement_angulaire_antihoraire();
  }else{
      deplacement();
  }
*/      deplacement();

  //mettre une condition sur l'angle_target, s'il est supérieu à 90° envoyer vers une fonction séparée

}

void deplacement(){

//si on utilise deplacement tout seul, il faut éxécute ces trois ligne une seule fois pour initialiser
robotX_previous = robotX;
robotY_previous = robotY;
robotTheta_previous = robotTheta;


erreur_distance = distance_target - sqrt((robotX-robotX_previous)*(robotX-robotX_previous) + (robotY-robotY_previous)*(robotY-robotY_previous));           //distance qu'il reste à parcourir
erreur_angle = angle_target - (robotTheta - robotTheta_previous);            //angle que le robot doit encore faire
//problème à régler pour le moment ou l'angle passe de -180° à 180° car l'erreur
//possible de passer sur une plage de valeur de 0° à 360° pour le régler, mais angle pas toujours optimal
if (erreur_distance > 0){
  Idistance_target += erreur_distance;
}else{                                      //technique secrète pour améliorer la stabilité de l'intégrateur
  Idistance_target = 0;
}


      cmdV = K_deplacement_prop * erreur_distance + K_deplacement_int * Idistance_target;
      cmd2  = (int)(cmdV*maxAcc);

      // on plafone a la vitesse de commande
      if(cmd2>maxSpeed){
          cmd2 = maxSpeed;
      }
      if(cmd2<-(maxSpeed)){
          cmd2 = -(maxSpeed);
      }
      cmd1 = cmd2;        //même commande pour gauche et droite

      // on limite l'acceleration
      if ((abs(cmd2)-abs(prevcmd2))>maxAcc){
          cmd2 = prevcmd2+ maxAcc* (cmd2-prevcmd2)/ abs(cmd2-prevcmd2);
      }
      if ((abs(cmd1)-abs(prevcmd1))>maxAcc){
          cmd1 = prevcmd1+ maxAcc* (cmd1-prevcmd1)/ abs(cmd1-prevcmd1);
      }
      
      prevcmd2 = cmd2;
      prevcmd1 = cmd1;

      if (abs(erreur_angle) < PI/40){                
        Iangle_target = 0;    //on reset l'intégrateur quand on est proche du but
      } 

      Iangle_target += erreur_angle;
     
      if (abs(erreur_angle) > PI/20){                //si angle trop grand, on fait pas de Ki car trop long sinon
          Ez = K_angle_prop * erreur_angle;
          Iangle_target = 0;
      }else{
        Ez = K_angle_prop * erreur_angle + K_angle_int * Iangle_target;   //diverge qaund l'erreur passe rapidement de -180 à 180
      }
      cmd2 += (int)(Ez*abs(cmd2));
      cmd1 -= (int)(Ez*abs(cmd1));


//diminution de la vitesse quand on approche de la cible
if (distance_target < 5){
  cmd1 = cmd1/2;
  cmd2 = cmd2/2;
}

  // Application des vitesses, uniquement si la distance est suffisante      (à tester si on refait un PID de vitesse dessus ou pas)
  if (distance_target > 2){
  set_vitesse(cmd1, cmd2);   
  }else{
    set_vitesse(0, 0);
  }


}


  void deplacement_angulaire_horaire() {
    //si on utilise deplacement tout seul, il faut éxécuter ces trois ligne une seule fois pour initialiser
    robotX_previous = robotX;
    robotY_previous = robotY;
    robotTheta_previous = robotTheta;
    
    
    erreur_angle = angle_target - (robotTheta - robotTheta_previous);            //angle que le robot doit encore faire
    //problème à régler pour le moment ou l'angle passe de -180° à 180° car l'erreur
    //possible de passer sur une plage de valeur de 0° à 360° pour le régler, mais angle pas toujours optimal
    if (erreur_distance > 0){
      Idistance_target += erreur_distance;
    }else{                                      //technique secrète pour améliorer la stabilité de l'intégrateur
      Idistance_target = 0;
    }
    

    if (abs(erreur_angle) < PI/20){                
      Iangle_target = 0;    //on reset l'intégrateur quand on est proche du but
    } 
          Iangle_target += erreur_angle;
         

          Ez = K_angle_prop * erreur_angle + K_angle_int * Iangle_target;   //diverge qaund l'erreur passe rapidement de -180 à 180
          
          cmd2 = Ez*10;
          cmd1 = -Ez*10;
    
  
      // Application des vitesses, uniquement si la distance est suffisante      (à tester si on refait un PID de vitesse dessus ou pas)
      if (distance_target > 2){
      set_vitesse(cmd1, cmd2);   
      }else{
        set_vitesse(0, 0);
      }
    //Serial.println("horaire");
    }


    void deplacement_angulaire_antihoraire() {
      //si on utilise deplacement tout seul, il faut éxécute ces trois ligne une seule fois pour initialiser
      robotX_previous = robotX;
      robotY_previous = robotY;
      robotTheta_previous = robotTheta;
      
      
      erreur_angle = angle_target - (robotTheta - robotTheta_previous);            //angle que le robot doit encore faire
      //problème à régler pour le moment ou l'angle passe de -180° à 180° car l'erreur
      //possible de passer sur une plage de valeur de 0° à 360° pour le régler, mais angle pas toujours optimal
      if (erreur_distance > 0){
        Idistance_target += erreur_distance;
      }else{                                      //technique secrète pour améliorer la stabilité de l'intégrateur
        Idistance_target = 0;
      }
      
      if (abs(erreur_angle) < PI/20){                
        Iangle_target = 0;    //on reset l'intégrateur quand on est proche du but
      } 
            Iangle_target += erreur_angle;
           


              Ez = K_angle_prop * erreur_angle + K_angle_int * Iangle_target;   //diverge qaund l'erreur passe rapidement de -180 à 180
            
            cmd2 = Ez*10;
            cmd1 = -Ez*10;
      
    
        // Application des vitesses, uniquement si la distance est suffisante      (à tester si on refait un PID de vitesse dessus ou pas)
        if (distance_target > 2){
        set_vitesse(cmd1, cmd2);   
        }else{
          set_vitesse(0, 0);
        }
        //Serial.println("antihoraire");

      }
      

void print_vitesse(){
  // Debug
  
  Serial.print("Vitesse1: ");
  Serial.print(speed1);
  //Serial.print(" | Commande1: ");
  //Serial.print(cmd1);
  Serial.print(" || Vitesse2: ");
  Serial.print(speed2);
  //Serial.print(" | Commande2: ");
  //Serial.print(cmd2);
  Serial.print(" || X: ");
  Serial.print(robotX);
  Serial.print(" || Y: ");
  Serial.print(robotY);
  Serial.print(" || theta: ");
  Serial.print(robotTheta * 180.0 / PI);
  Serial.print(" || distance_target: ");
  Serial.print(distance_target);
  Serial.print(" || angle_target: ");
  Serial.print(angle_target * 180.0 / PI);
  Serial.print(" || Ez: ");
  Serial.print(Ez);
  Serial.print(" || TargetTheta: ");
  Serial.println(targetTheta * 180.0 / PI);

  //Serial.print(" || Distance: ");
  //Serial.print(sqrt(robotX*robotX + robotY*robotY));
  //Serial.print(" || erreur_angle: ");
  //Serial.println(erreur_angle * 180.0 / PI);
//  Serial.print(" Destination ?");
//Serial.println(destination_atteinte);
}


void computePosition(float deltaLeft, float deltaRight) {
  // Calcul du déplacement linéaire moyen et de la rotation angulaire
  deltaLeft = deltaLeft*(duree_pid/1000)*3.3;       //pour adapter le calcul de la position avec la durée de calcul du PID
  deltaRight = deltaRight*(duree_pid/1000)*3.3;
  float deltaDistance = (deltaLeft + deltaRight) / 2.0;
  float deltaTheta = (deltaRight - deltaLeft) / wheelBase;

  // Mise à jour de l'orientation
  robotTheta += deltaTheta;

  if (robotTheta > PI) {
    robotTheta -= 2*PI;
  } else if (robotTheta < -PI) {
    robotTheta +=  2*PI;
  }
  // Mise à jour des positions X et Y
  robotX += deltaDistance * cos(robotTheta);
  robotY += deltaDistance * sin(robotTheta);
}


void IRAM_ATTR onServoTimer() {
  portENTER_CRITICAL_ISR(&servoTimerMux); // Protection contre les interruptions concurrentes

  // Mise à jour de l'angle
  angleServo += stepServo;

  // Vérifier si on atteint une limite et inverser la direction
  if (angleServo <= angleMin || angleServo >= angleMax) {
    stepServo = -stepServo;  // Inversion du mouvement
  }

  myServo.write(angleServo);  // Appliquer le nouvel angle

  portEXIT_CRITICAL_ISR(&servoTimerMux); // Fin de la protection
}

void setupServoInterrupt() {
  // Configuration du timer hardware pour une interruption toutes les `servoInterval` ms
  servoTimer = timerBegin(1, 80, true); // Timer 1, prescaler 80 (1 µs par tick), compteur incrémental
  timerAttachInterrupt(servoTimer, &onServoTimer, true); // Attacher l'interruption au timer
  timerAlarmWrite(servoTimer, servoInterval * 1000, true); // Déclencher l'interruption toutes les `servoInterval` ms
  timerAlarmEnable(servoTimer); // Activer l'alarme du timer
}



bool arrivee_destination(){
  if ((abs (robotX - targetX) < 5) && (abs (robotY - targetY) < 5)){
    destination_atteinte = true;
  }
  else{
    destination_atteinte = false;
  }
  return destination_atteinte;
}

bool arrivee_destination_finale(){
  if (((abs (robotX - coordinates_temp_jaune[numCoordinates-1][0]) < 5) && (abs (robotY - coordinates_temp_jaune[numCoordinates-1][1]) < 5)) || ((abs (robotX - coordinates_temp_bleu[numCoordinates-1][0]) < 5) && (abs (robotY - coordinates_temp_bleu[numCoordinates-1][1]) < 5))){
    destination_finale_atteinte = true;
    //Serial.println("destination_atteinte");
  }
  //else{
  //  destination_atteinte = false;
  //}
  Serial.print("destination :");
  Serial.println(destination_finale_atteinte);
  return destination_finale_atteinte;
}

void asser_position_loop(bool stop){
  unsigned long currentTime = millis();



  if (timerFlag) {
    timerFlag = false; // Réinitialiser le flag
  
  //Calcul vitesse 
  speed1 = ((pulseCount1 / (float)pulsesPerRevolution) * directionSpeed1)*(1000/duree_pid);         //coeff de 1000:duree_pid car on fait le calcul tout les 1/duree_pid secondes, et on veut la valeur en tour/s
  speed2 = ((pulseCount2 / (float)pulsesPerRevolution) * directionSpeed2)*(1000/duree_pid);

  pulseCount1 = 0; // Réinitialisation des impulsions
  pulseCount2 = 0;

  float deltaLeft = speed1*wheelCircumference*2;
  float deltaRight = speed2* wheelCircumference*2;

      // Calcul de la position
  computePosition(deltaLeft, deltaRight);
  
  if(stop){
    if (arrivee_destination_finale() == false){
      go_to();  // Déplacement vers la destination, lorsqu'il n'y a pas d'obstacle
    }else{
    set_vitesse(0,0);
    //Serial.println("stop");
  }
  }else if((distance1 < 3) || (distance2 < 3) || (distance_tof1 < 3) || (distance_tof2 < 3) || (distance_tof3 < 3) || (distance_tof4 < 3)){
    // Si un obstacle est détecté trop près, arrêter le robot
    set_vitesse(0, 0);
  }else{
    set_vitesse(0, 0); // Arrêter les moteurs si le robot est arrêté
  }
    
  if (arrivee_destination() == true && millis() - lastArrivalTime >= 1000){       //condition pour changer de coordonnée cible
    lastArrivalTime = millis(); // Mettre à jour le temps de la dernière arrivée

    if (i == numCoordinates-1){
      i = i;
    }else{
      i++;
    }
    if (couleur == true){
      targetX = coordinates_temp_jaune[i][0];
      targetY = coordinates_temp_jaune[i][1];
    }else{
      targetX = coordinates_temp_bleu[i][0];
      targetY = coordinates_temp_bleu[i][1];
    }
  lastTime1 = currentTime;
  
  }
  //print_vitesse();
  }
}

