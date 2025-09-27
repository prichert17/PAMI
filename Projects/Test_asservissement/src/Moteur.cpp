#include <Arduino.h>
// Définition des pins pour le moteur 1 = moteur droit (STM32 L432KC)
const int motorPin1 = PA6;  // Pin du moteur 1 (PWM sens 1) - TIM3_CH1
const int motorPin2 = PA7;  // Pin du moteur 1 (PWM sens 2) - TIM3_CH2
const int encoderPinA = PA8; // Pin de l'encodeur 1 A - TIM1_CH1
const int encoderPinB = PA9; // Pin de l'encodeur 1 B - TIM1_CH2

// Définition des pins pour le moteur 2 = moteur gauche (STM32 L432KC)
const int motorPin3 = PA10;  // Pin du moteur 2 (PWM sens 1) - TIM1_CH3
const int motorPin4 = PA11;  // Pin du moteur 2 (PWM sens 2) - TIM1_CH4
const int encoderPinC = PB3; // Pin de l'encodeur 2 A - TIM2_CH2
const int encoderPinD = PB4; // Pin de l'encodeur 2 B - TIM3_CH1

// Variables pour la gestion de l'encodeur 1
volatile int pulseCount1 = 0;    // Nombre d'impulsions pour le moteur 1
unsigned long lastTime1 = 0;     // Temps du dernier calcul pour le moteur 1
float speed1 = 0.0;              // Vitesse du moteur 1 en tours par seconde
int directionSpeed1 = 1;         // Direction du moteur 1

// Variables pour la gestion de l'encodeur 2
volatile int pulseCount2 = 0;    // Nombre d'impulsions pour le moteur 2
float speed2 = 0.0;              // Vitesse du moteur 2 en tours par seconde
int directionSpeed2 = 1;         // Direction du moteur 2

// Nombre d'impulsions par tour (selon la fiche technique)
const int pulsesPerRevolution = 1400;  // Valeur mise à jour

// Paramètres pour le PID
float kp = 0.5, ki = 0.3, kd = -0.2;
float targetSpeed1 = 255; // Vitesse cible moteur 1
float targetSpeed2 = 255; // Vitesse cible moteur 2
float error1 = 0, error2 = 0;
float prevError1 = 0, prevError2 = 0;
float integral1 = 0, integral2 = 0;
float duree_pid = 100;          //duree du PID en ms (intervalle de temps entre deux calcul)


// Déclaration des variables supplémentaires pour la position du robot
float robotX = 0.0;  // Position X du robot en cm
float robotY = 0.0;  // Position Y du robot en cm
float robotTheta = 0.0;  // Orientation du robot en radians

// Diamètre des roues en cm
const float wheelDiameter = 4.30;  
const float wheelCircumference = wheelDiameter * PI;

// Distance entre les roues en cm (les centres des deux roues, cf modélisation 3D)
const float wheelBase = 7.60;

// Déclaration des fonctions
void countPulse1();
void countPulse2();
void set_vitesse(float cmd1, float cmd2);
void afficher_vitesse_instantannee();
void computePID();
void computePosition(float deltaLeft, float deltaRight);



void setup() {
  // Initialisation des pins pour le moteur 1
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);  // Pullup pour stabilité des signaux encodeur
  pinMode(encoderPinB, INPUT_PULLUP);

  // Initialisation des pins pour le moteur 2
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(encoderPinC, INPUT_PULLUP);  // Pullup pour stabilité des signaux encodeur
  pinMode(encoderPinD, INPUT_PULLUP);

  // Initialisation de la communication série
  Serial.begin(115200);

  // Attache les interruptions pour les encodeurs
  attachInterrupt(digitalPinToInterrupt(encoderPinA), countPulse1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinC), countPulse2, RISING);

  // Applique la vitesse initiale
  set_vitesse(targetSpeed1,targetSpeed2);
  Serial.println(" ");
}

void loop() {
  unsigned long currentTime = millis();

  // Calcul des déplacements des roues (delta en cm/s)
    float deltaLeft = speed1*wheelCircumference*2;
    float deltaRight = speed2* wheelCircumference*2;
  //Serial.println(deltaLeft);
  //Serial.println(deltaRight);


  

  // Calcul de la vitesse pour le moteur 1 toutes les 100 ms (0,1 seconde)
  if (currentTime - lastTime1 >= duree_pid) {
    //afficher_vitesse_instantannee();
    computePID();
    // Calcul de la position
    computePosition(deltaLeft, deltaRight);
    lastTime1 = currentTime;
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
  // Contrainte des valeurs entre -255 et 255
  cmd1 = constrain(cmd1, -255.0f, 255.0f);
  cmd2 = constrain(cmd2, -255.0f, 255.0f);
  
  // Moteur 1
  if (cmd1 > 0) {
    analogWrite(motorPin1, (int)abs(cmd1));
    analogWrite(motorPin2, 0);
  } else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, (int)abs(cmd1));
  }

  // Moteur 2
  if (cmd2 > 0) {
    analogWrite(motorPin3, (int)abs(cmd2));
    analogWrite(motorPin4, 0);
  } else {
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, (int)abs(cmd2));
  }
}


// Fonction pour afficher les vitesses instantanées des deux moteurs        //fonction inutilisée
void afficher_vitesse_instantannee() {
   // Moteur 1
  speed1 = (227*(pulseCount1 / (float)pulsesPerRevolution) * directionSpeed1)*(1000/duree_pid);         //coeff de 1000/duree_pid car on fait le calcul tout les 1/duree_pid secondes, et on veut la valeur en tour/s pour ensuite la passer en valeur entre [0;255] avec le coeff 227
  speed2 = (227*(pulseCount2 / (float)pulsesPerRevolution) * directionSpeed2)*(1000/duree_pid);
  set_vitesse(speed1,speed2);


  Serial.print("|   1    | ");
  Serial.print(speed1, 2);  // Affiche avec 2 décimales
  Serial.print("         | ");
  Serial.print(directionSpeed1 > 0 ? "Avant     " : "Arrière   ");
  Serial.print("| ");
  Serial.print(error1);
  Serial.println("                   |");

  // Moteur 2
  Serial.print("|   2    | ");
  Serial.print(speed2, 2);  // Affiche avec 2 décimales
  Serial.print("         | ");
  Serial.print(directionSpeed2 > 0 ? "Avant     " : "Arrière   ");
  Serial.print("| ");
  Serial.print(error2);
  Serial.println("                   |");

pulseCount1 = 0; // Réinitialisation des impulsions
  pulseCount2 = 0;
}



// Fonction pour calculer le PID
void computePID() {

  /* Calcul des erreurs */

    // Moteur 1
  speed1 = ((pulseCount1 / (float)pulsesPerRevolution) * directionSpeed1)*(1000/duree_pid);         //coeff de 1000:duree_pid car on fait le calcul tout les 1/duree_pid secondes, et on veut la valeur en tour/s
  error1 = (targetSpeed1 - speed1*227);

  // Moteur 2
  speed2 = ((pulseCount2 / (float)pulsesPerRevolution) * directionSpeed2)*(1000/duree_pid);
  error2 = (targetSpeed2 - speed2*227);


  pulseCount1 = 0; // Réinitialisation des impulsions
  pulseCount2 = 0;

  // Intégrale
  integral1 += error1;
  integral2 += error2;

  // Dérivée
  float derivative1 = error1 - prevError1;
  float derivative2 = error2 - prevError2;

  // Calcul des commandes
  float cmd1 = kp * error1 + ki * integral1 + kd * derivative1;
  float cmd2 = kp * error2 + ki * integral2 + kd * derivative2;

  // Sauvegarde des erreurs précédentes
  prevError1 = error1;
  prevError2 = error2;

  // Application des vitesses
  set_vitesse(cmd1, cmd2);


  // Debug
  
  Serial.print("Vitesse1: ");
  Serial.print(speed1);
  Serial.print(" | Commande1: ");
  Serial.print(cmd1);
  Serial.print(" || Vitesse2: ");
  Serial.print(speed2);
  Serial.print(" | Commande2: ");
  Serial.print(cmd2);
  Serial.print(" || error: ");
  Serial.print(abs(error1));
  Serial.print(" | ");
  Serial.print(abs(error2));
  Serial.print(" || X: ");
  Serial.print(robotX);
  Serial.print(" || Y: ");
  Serial.print(robotY);
  Serial.print(" || theta: ");
  Serial.println(robotTheta * 180.0 / PI);

}


void computePosition(float deltaLeft, float deltaRight) {
  // Calcul du déplacement linéaire moyen et de la rotation angulaire
  deltaLeft = deltaLeft*(duree_pid/1000);       //pour adapter le calcul de la position avec la durée de calcul du PID
  deltaRight = deltaRight*(duree_pid/1000);
  float deltaDistance = (deltaLeft + deltaRight) / 2.0;
  float deltaTheta = (deltaRight - deltaLeft) / wheelBase;

  // Mise à jour de l'orientation
  robotTheta += deltaTheta;

  // Mise à jour des positions X et Y
  robotX += deltaDistance * cos(robotTheta);
  robotY += deltaDistance * sin(robotTheta);
}