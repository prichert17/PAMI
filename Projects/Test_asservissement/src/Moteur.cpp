#include <Arduino.h>

// HARDWARE TIMER - STM32 L432KC
// TIM6 : Timer contrôle PID
HardwareTimer *tCtrl = new HardwareTimer(TIM6);
// Note: TIM2 et TIM3 seront configurés plus tard pour encodeurs hardware

// Pins moteurs
const uint8_t motorPin1 = PA8;  // Moteur gauche sens 1
const uint8_t motorPin2 = PA9;  // Moteur gauche sens 2  
const uint8_t motorPin3 = PA10; // Moteur droit sens 1
const uint8_t motorPin4 = PA11; // Moteur droit sens 2

// Variables encodeurs temporaires (en attendant config hardware)
volatile int32_t enc1Count = 0;
volatile int32_t enc2Count = 0;
volatile uint8_t lastStateEnc1 = 0;
volatile uint8_t lastStateEnc2 = 0;

// Variables vitesses et commandes
volatile float speed1 = 0.0;
volatile float speed2 = 0.0;
float cmd1 = 0.0;
float cmd2 = 0.0;

// Paramètres système
const int pulsesPerRevolution = 1200;  // CPR encodeur
const float Te = 0.001f;               // Période PID (1ms = 1kHz)

// ISR encodeur temporaires (mode GPIO)
void enc1_ISR() {
  uint8_t newState = (digitalRead(PA1) << 1) | digitalRead(PA0);
  uint8_t transition = (lastStateEnc1 << 2) | newState;
  
  switch(transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: enc1Count++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: enc1Count--; break;
  }
  lastStateEnc1 = newState;
}

void enc2_ISR() {
  uint8_t newState = (digitalRead(PA7) << 1) | digitalRead(PA6);
  uint8_t transition = (lastStateEnc2 << 2) | newState;
  
  switch(transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: enc2Count++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: enc2Count--; break;
  }
  lastStateEnc2 = newState;
}

// Configuration encodeurs (temporaire mode GPIO, à migrer vers hardware)
void setupEncoders() {
  // Configuration pins encodeurs en GPIO avec pullup
  pinMode(PA0, INPUT_PULLUP); // Encodeur gauche A
  pinMode(PA1, INPUT_PULLUP); // Encodeur gauche B
  pinMode(PA6, INPUT_PULLUP); // Encodeur droit A
  pinMode(PA7, INPUT_PULLUP); // Encodeur droit B
  
  // État initial
  lastStateEnc1 = (digitalRead(PA1) << 1) | digitalRead(PA0);
  lastStateEnc2 = (digitalRead(PA7) << 1) | digitalRead(PA6);
  
  // Interruptions GPIO (temporaire)
  attachInterrupt(digitalPinToInterrupt(PA0), enc1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PA1), enc1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PA6), enc2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PA7), enc2_ISR, CHANGE);
  
  enc1Count = 0;
  enc2Count = 0;
  
  Serial.println("Encodeurs configurés (mode GPIO temporaire)");
}

// Lecture delta encodeurs depuis dernière période PID
inline int32_t readAndZeroEnc1() { 
  noInterrupts();
  int32_t c = enc1Count;
  enc1Count = 0;
  interrupts();
  return c; 
}

inline int32_t readAndZeroEnc2() { 
  noInterrupts();
  int32_t c = enc2Count;
  enc2Count = 0;
  interrupts();
  return c; 
}

// Fonction moteurs
void setMotors(float c1, float c2) {
  cmd1 = constrain(c1, -255, 255);
  cmd2 = constrain(c2, -255, 255);
  
  // Moteur 1
  if (cmd1 > 0) {
    analogWrite(motorPin1, (int)cmd1);
    analogWrite(motorPin2, 0);
  } else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, (int)abs(cmd1));
  }

  // Moteur 2
  if (cmd2 > 0) {
    analogWrite(motorPin3, (int)cmd2);
    analogWrite(motorPin4, 0);
  } else {
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, (int)abs(cmd2));
  }
}

// ISR Timer contrôle PID (1kHz)
void ctrlISR() {
  // Lecture delta encodeurs depuis dernière période
  int32_t delta1 = readAndZeroEnc1(); // Encodeur gauche
  int32_t delta2 = readAndZeroEnc2(); // Encodeur droit
  
  // Conversion en vitesses (tr/s)
  speed1 = delta1 / (pulsesPerRevolution * Te);
  speed2 = delta2 / (pulsesPerRevolution * Te);
  
  // Commandes simples pour test
  float cmd1_test = 0;   // Moteur gauche arrêté
  float cmd2_test = 80;  // Moteur droit vitesse fixe
  
  setMotors(cmd1_test, cmd2_test);
}

void setup() {
  Serial.begin(115200);
  Serial.println("STM32 Hardware Timer Encodeurs - Test");
  
  // Configuration pins moteurs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  // Configuration encodeurs hardware
  setupEncoders();
  
  // Configuration timer contrôle PID à 1kHz
  tCtrl->setOverflow(1000, HERTZ_FORMAT);
  tCtrl->attachInterrupt(ctrlISR);
  tCtrl->resume();
  
  Serial.println("Système prêt - Timer PID 1kHz actif");
  
  // Moteurs arrêtés au démarrage
  setMotors(0, 0);
}

void loop() {
  // Boucle principale - affichage debug seulement
  static unsigned long lastDebug = 0;
  
  if (millis() - lastDebug > 500) { // Debug toutes les 500ms
    // Capture sécurisée des variables volatiles
    noInterrupts();
    float localSpeed1 = speed1;
    float localSpeed2 = speed2;
    interrupts();
    
    // Affichage debug
    noInterrupts();
    int32_t currentEnc1 = enc1Count;
    int32_t currentEnc2 = enc2Count;
    interrupts();
    
    Serial.print("V1: "); Serial.print(localSpeed1, 3);
    Serial.print(" | V2: "); Serial.print(localSpeed2, 3);
    Serial.print(" | C1: "); Serial.print(cmd1, 0);
    Serial.print(" | C2: "); Serial.print(cmd2, 0);
    Serial.print(" | E1: "); Serial.print(currentEnc1);
    Serial.print(" | E2: "); Serial.println(currentEnc2);
    
    lastDebug = millis();
  }
  
  delay(10); // Évite une boucle trop agressive
}