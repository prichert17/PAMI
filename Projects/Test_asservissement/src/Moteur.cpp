#include <Arduino.h>

// Pins moteurs - STM32 L432KC
const uint8_t motorPin1 = PA8;  // Moteur gauche sens 1
const uint8_t motorPin2 = PA9;  // Moteur gauche sens 2  
const uint8_t motorPin3 = PA10; // Moteur droit sens 1
const uint8_t motorPin4 = PA11; // Moteur droit sens 2

// Pins encodeurs
const uint8_t encA1 = PA0; // Encodeur gauche A 
const uint8_t encB1 = PA1; // Encodeur gauche B
const uint8_t encA2 = PA7; // Encodeur droit A  
const uint8_t encB2 = PA6; // Encodeur droit B

// Variables encodeurs
volatile int32_t enc1Count = 0;
volatile int32_t enc2Count = 0;
volatile uint8_t lastStateEnc1 = 0;
volatile uint8_t lastStateEnc2 = 0;

// Variables vitesses
float speed1 = 0.0;
float speed2 = 0.0;

// Variables commandes
float cmd1 = 0.0;
float cmd2 = 0.0;

// ISR encodeur 1
void enc1_ISR() {
  uint8_t newState = (digitalRead(encB1) << 1) | digitalRead(encA1);
  uint8_t transition = (lastStateEnc1 << 2) | newState;
  
  switch(transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: enc1Count++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: enc1Count--; break;
  }
  lastStateEnc1 = newState;
}

// ISR encodeur 2
void enc2_ISR() {
  uint8_t newState = (digitalRead(encB2) << 1) | digitalRead(encA2);
  uint8_t transition = (lastStateEnc2 << 2) | newState;
  
  switch(transition) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: enc2Count++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: enc2Count--; break;
  }
  lastStateEnc2 = newState;
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

void setup() {
  Serial.begin(115200);
  
  // Configuration pins moteurs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  // Configuration pins encodeurs
  pinMode(encA1, INPUT_PULLUP);
  pinMode(encB1, INPUT_PULLUP);
  pinMode(encA2, INPUT_PULLUP);
  pinMode(encB2, INPUT_PULLUP);
  
  // État initial encodeurs
  lastStateEnc1 = (digitalRead(encB1) << 1) | digitalRead(encA1);
  lastStateEnc2 = (digitalRead(encB2) << 1) | digitalRead(encA2);
  
  // Interruptions encodeurs
  attachInterrupt(digitalPinToInterrupt(encA1), enc1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB1), enc1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA2), enc2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB2), enc2_ISR, CHANGE);
  
  Serial.println("STM32 Test Moteurs MINIMAL - Prêt");
  
  // Moteurs arrêtés
  setMotors(0, 0);
}

void loop() {
  static unsigned long lastTime = 0;
  static int32_t lastEnc1 = 0;
  static int32_t lastEnc2 = 0;
  
  if (millis() - lastTime > 100) { // Toutes les 100ms
    
    // Calcul vitesses simples
    noInterrupts();
    int32_t currentEnc1 = enc1Count;
    int32_t currentEnc2 = enc2Count;
    interrupts();
    
    int32_t delta1 = currentEnc1 - lastEnc1;
    int32_t delta2 = currentEnc2 - lastEnc2;
    
    speed1 = delta1 * 10.0 / 1200.0; // tr/s approximatif
    speed2 = delta2 * 10.0 / 1200.0;
    
    lastEnc1 = currentEnc1;
    lastEnc2 = currentEnc2;
    
    // Test simple : moteur droit à vitesse fixe, gauche arrêté
    setMotors(0, 100); // M1=0, M2=100
    
    // Debug
    Serial.print("V1: "); Serial.print(speed1, 2);
    Serial.print(" | V2: "); Serial.print(speed2, 2);
    Serial.print(" | C1: "); Serial.print(cmd1, 0);
    Serial.print(" | C2: "); Serial.print(cmd2, 0);
    Serial.print(" | E1: "); Serial.print(currentEnc1);
    Serial.print(" | E2: "); Serial.println(currentEnc2);
    
    lastTime = millis();
  }
  
  delay(10); // Délai pour stabilité
}