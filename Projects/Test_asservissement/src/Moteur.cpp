#include <Arduino.h>

// Mapping pins - STM32 L432KC
// Moteurs PWM (TIM1 20kHz)
const uint8_t motorPin1 = PA8;  // TIM1_CH1 - Moteur gauche sens 1
const uint8_t motorPin2 = PA9;  // TIM1_CH2 - Moteur gauche sens 2  
const uint8_t motorPin3 = PA10; // TIM1_CH3 - Moteur droit sens 1
const uint8_t motorPin4 = PA11; // TIM1_CH4 - Moteur droit sens 2

// Encodeurs (GPIO avec timer périodique pour lecture)
const uint8_t encA1 = PA0; // Encodeur gauche A 
const uint8_t encB1 = PA1; // Encodeur gauche B
const uint8_t encA2 = PA6; // Encodeur droit A  
const uint8_t encB2 = PA7; // Encodeur droit B

// UART vers ESP32 (PA2/PA3 - USART2 → Serial2)

// Timer pour contrôle PID
HardwareTimer *tCtrl = new HardwareTimer(TIM6);  // Timer contrôle PID

// Variables pour la gestion des encodeurs en quadrature
volatile int32_t enc1Count = 0;   // Compteur encodeur gauche
volatile int32_t enc2Count = 0;   // Compteur encodeur droit
volatile int32_t lastEnc1 = 0;    // Dernière valeur encodeur gauche
volatile int32_t lastEnc2 = 0;    // Dernière valeur encodeur droit  
volatile float speed1 = 0.0;      // Vitesse moteur gauche en tr/s
volatile float speed2 = 0.0;      // Vitesse moteur droit en tr/s

// États précédents pour décodage quadrature
volatile uint8_t lastStateEnc1 = 0;
volatile uint8_t lastStateEnc2 = 0;

// Nombre d'impulsions par tour (selon la fiche technique)
const int pulsesPerRevolution = 1400;  // CPR réducteur
const float Te = 0.001f;               // Période échantillonnage (1kHz)

// Paramètres pour le PID (conservés identiques)
float kp = 0.5, ki = 0.3, kd = -0.2;
float targetSpeed1 = 255; // Vitesse cible moteur gauche (PWM equivalent)
float targetSpeed2 = 255; // Vitesse cible moteur droit (PWM equivalent)
float error1 = 0, error2 = 0;
float prevError1 = 0, prevError2 = 0;
float integral1 = 0, integral2 = 0;

// Variables supplémentaires pour la position du robot (conservées)
float robotX = 0.0;       // Position X du robot en cm
float robotY = 0.0;       // Position Y du robot en cm
float robotTheta = 0.0;   // Orientation du robot en radians

// Diamètre des roues en cm (conservé)
const float wheelDiameter = 4.30;  
const float wheelCircumference = wheelDiameter * PI;

// Distance entre les roues en cm (conservé)
const float wheelBase = 7.60;

// Déclaration des fonctions
void setupPWM_20k();
void setupEncoders(); 
void setupCtrlLoopTimer(uint32_t freq_hz);
int32_t readAndZeroEnc1(); // Lecture différentielle encodeur gauche
int32_t readAndZeroEnc2(); // Lecture différentielle encodeur droit
void ctrlISR();            // ISR contrôle PID 
void enc1_ISR();           // ISR encodeur 1
void enc2_ISR();           // ISR encodeur 2  
void set_vitesse(float cmd1, float cmd2); // Fonction PWM (signature conservée)
void computePosition(float deltaLeft, float deltaRight); // Calcul position



void setup() {
  // Initialisation communication série (USART2 PA2/PA3)
  Serial2.begin(115200);
  Serial.begin(115200); // USB Serial pour debug
  
  // Configuration PWM 20kHz 
  setupPWM_20k();
  
  // Configuration encodeurs avec interruptions
  setupEncoders();
  
  // Configuration timer contrôle PID à 1kHz
  setupCtrlLoopTimer(1000);
  
  Serial.println("STM32 L432KC - Controle moteurs avec encodeurs quadrature");
  Serial2.println("INIT,OK");
}

void loop() {
  // Boucle principale vide - contrôle via ISR timer
  // Optionnel: debug périodique par Serial
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) { // Debug toutes les 500ms
    Serial.print("V1: "); Serial.print(speed1, 3);
    Serial.print(" | V2: "); Serial.print(speed2, 3);
    Serial.print(" | X: "); Serial.print(robotX, 1);
    Serial.print(" | Y: "); Serial.print(robotY, 1);
    Serial.print(" | θ: "); Serial.println(robotTheta * 180.0 / PI, 1);
    
    // Envoi via UART vers ESP32 (format CSV)
    Serial2.print(speed1); Serial2.print(",");
    Serial2.print(speed2); Serial2.print(",");
    Serial2.print(robotX); Serial2.print(",");
    Serial2.print(robotY); Serial2.print(",");
    Serial2.println(robotTheta * 180.0 / PI);
    
    lastDebug = millis();
  }
}

// === FONCTIONS DE CONFIGURATION ===

// Configuration PWM 20kHz sur pins moteurs 
void setupPWM_20k() {
  // Configure pins en mode PWM
  pinMode(motorPin1, OUTPUT); // PA8
  pinMode(motorPin2, OUTPUT); // PA9
  pinMode(motorPin3, OUTPUT); // PA10
  pinMode(motorPin4, OUTPUT); // PA11
  
  // Configure TIM1 pour 20kHz (nécessite accès registres si pas dans framework)
  // Pour l'instant utilisation analogWrite standard, fréquence par défaut ~1kHz
  Serial.println("PWM configuré sur PA8..PA11 (fréquence par défaut)");
}

// Configuration encodeurs avec interruptions GPIO
void setupEncoders() {
  // Configuration pins encodeur 1 (gauche)
  pinMode(encA1, INPUT_PULLUP); // PA0
  pinMode(encB1, INPUT_PULLUP); // PA1
  
  // Configuration pins encodeur 2 (droit)  
  pinMode(encA2, INPUT_PULLUP); // PA6
  pinMode(encB2, INPUT_PULLUP); // PA7
  
  // État initial
  lastStateEnc1 = (digitalRead(encB1) << 1) | digitalRead(encA1);
  lastStateEnc2 = (digitalRead(encB2) << 1) | digitalRead(encA2);
  
  // Interruptions sur fronts (A et B pour chaque encodeur)
  attachInterrupt(digitalPinToInterrupt(encA1), enc1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB1), enc1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA2), enc2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB2), enc2_ISR, CHANGE);
  
  enc1Count = 0;
  enc2Count = 0;
  lastEnc1 = 0;
  lastEnc2 = 0;
  Serial.println("Encodeurs configurés avec interruptions GPIO");
}

// Configuration timer contrôle PID
void setupCtrlLoopTimer(uint32_t freq_hz) {
  tCtrl->setOverflow(freq_hz, HERTZ_FORMAT);
  tCtrl->attachInterrupt(ctrlISR);
  tCtrl->resume();
  Serial.print("Timer contrôle PID configuré à "); 
  Serial.print(freq_hz); Serial.println(" Hz");
}

// === ISR ENCODEURS QUADRATURE ===

// ISR encodeur 1 (gauche) - décodage quadrature X4
void enc1_ISR() {
  uint8_t newState = (digitalRead(encB1) << 1) | digitalRead(encA1);
  uint8_t transition = (lastStateEnc1 << 2) | newState;
  
  // Table de décodage quadrature (sens + comptage X4)
  switch(transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: enc1Count++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: enc1Count--; break;
  }
  lastStateEnc1 = newState;
}

// ISR encodeur 2 (droit) - décodage quadrature X4  
void enc2_ISR() {
  uint8_t newState = (digitalRead(encB2) << 1) | digitalRead(encA2);
  uint8_t transition = (lastStateEnc2 << 2) | newState;
  
  // Table de décodage quadrature (sens + comptage X4)
  switch(transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: enc2Count++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: enc2Count--; break;
  }
  lastStateEnc2 = newState;
}

// === FONCTIONS DE LECTURE ENCODEURS ===

// Lecture différentielle encodeur gauche
int32_t readAndZeroEnc1() {
  noInterrupts();
  int32_t current = enc1Count;
  int32_t delta = current - lastEnc1;
  lastEnc1 = current;
  interrupts();
  return delta;
}

// Lecture différentielle encodeur droit
int32_t readAndZeroEnc2() {
  noInterrupts();
  int32_t current = enc2Count;
  int32_t delta = current - lastEnc2;
  lastEnc2 = current;
  interrupts();
  return delta;
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


// === FONCTION ISR CONTROLE PID ===

// ISR appelée par timer à 1kHz pour contrôle PID
void ctrlISR() {
  // Lecture différentielle des encodeurs 
  int32_t delta1 = readAndZeroEnc1(); // Encodeur gauche
  int32_t delta2 = readAndZeroEnc2(); // Encodeur droit
  
  // Conversion en vitesses (tr/s)
  speed1 = delta1 / (pulsesPerRevolution * Te); // Vitesse gauche
  speed2 = delta2 / (pulsesPerRevolution * Te); // Vitesse droite
  
  // Calcul erreurs PID (cible en equivalent PWM, vitesse en tr/s * 227)
  error1 = targetSpeed1 - (speed1 * 227.0f);
  error2 = targetSpeed2 - (speed2 * 227.0f);
  
  // Intégration
  integral1 += error1;
  integral2 += error2;
  
  // Dérivation  
  float derivative1 = error1 - prevError1;
  float derivative2 = error2 - prevError2;
  
  // Commandes PID
  float cmd1 = kp * error1 + ki * integral1 + kd * derivative1;
  float cmd2 = kp * error2 + ki * integral2 + kd * derivative2;
  
  // Sauvegarde erreurs précédentes
  prevError1 = error1;
  prevError2 = error2;
  
  // Application commandes moteurs (signature conservée)
  set_vitesse(cmd1, cmd2);
  
  // Calcul position (odométrie)
  float deltaLeft = speed1 * wheelCircumference * Te;   // Distance parcourue roue gauche
  float deltaRight = speed2 * wheelCircumference * Te;  // Distance parcourue roue droite
  computePosition(deltaLeft, deltaRight);
}


// === FONCTION CALCUL POSITION (ODOMÉTRIE) ===

void computePosition(float deltaLeft, float deltaRight) {
  // Calcul du déplacement linéaire moyen et de la rotation angulaire
  // deltaLeft/deltaRight sont déjà en cm (calculés avec Te dans ctrlISR)
  float deltaDistance = (deltaLeft + deltaRight) / 2.0;
  float deltaTheta = (deltaRight - deltaLeft) / wheelBase;

  // Mise à jour de l'orientation
  robotTheta += deltaTheta;

  // Mise à jour des positions X et Y
  robotX += deltaDistance * cos(robotTheta);
  robotY += deltaDistance * sin(robotTheta);
}