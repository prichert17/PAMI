/* NUCLEO-G431KB - RÉGULATION PID MOTEUR SIMPLE
 * PID en boucle fermée avec un seul moteur + encodeur TIM2
 * Moteur: D9=PA8 (+), D3=PB0 (-)
 * Encodeur: A0=PA0 (TIM2_CH1), A1=PA1 (TIM2_CH2)
 */

#include <Arduino.h>
extern "C" {
  #include "stm32g4xx_hal.h"  // Changé de stm32l4xx_hal.h
}

// Pins PWM pour moteur (seulement le droit)
const uint8_t M_PWM_P = D9;   // TIM1_CH1  (PA8)  - Moteur +
const uint8_t M_PWM_N = D3;   // TIM1_CH2N (PB0)  - Moteur -

// Encodeur TIM2 (moteur droit)
TIM_HandleTypeDef htim2;
// Encodeur TIM3 (moteur gauche - D11/D12)
TIM_HandleTypeDef htim3;
static constexpr uint32_t ENC_MID = 0x7FFFFFFFUL;

// Paramètres encodeur
static constexpr float CPR_MOTOR = 1200.0f;  // Impulsions par tour moteur
static constexpr float GEAR_RATIO = 100.0f;  // Réducteur 100:1
static constexpr float QUAD_FACTOR = 4.0f;   // Mode encodeur X4
static constexpr float TICKS_PER_REV = (CPR_MOTOR * QUAD_FACTOR) / GEAR_RATIO; // = 48 ticks/tour roue

// Paramètres PID
static constexpr float DT_S = 0.050f; // 50ms période PID

// Gains PID (ajustés pour votre moteur)
float kp = 0.8f;  // Proportionnel
float ki = 0.2f;  // Intégral  
float kd = 0.1f;  // Dérivée

// Variables PID
float targetSpeed_rps = 0.0f;  // Consigne vitesse en tr/s
float currentSpeed_rps = 0.0f; // Vitesse mesurée
float error = 0.0f, errorPrev = 0.0f, errorSum = 0.0f;
float pidOutput = 0.0f;

// Variables encodeur moteur droit
int32_t encoderCount = 0, lastEncoderCount = 0;
// Variables encodeur moteur gauche
int32_t encoderCountLeft = 0, lastEncoderCountLeft = 0;
float currentSpeedLeft_rps = 0.0f;
unsigned long compteur = 0;

// Configuration encodeur TIM2
void MX_TIM2_Encoder_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct{};
  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig{};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 2;
  sConfig.IC2Polarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 2;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler         = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 0xFFFFFFFFUL;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.RepetitionCounter = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim2, &sConfig);
  __HAL_TIM_SET_COUNTER(&htim2, ENC_MID);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

// Configuration encodeur TIM3 (moteur gauche - D11/D12)
void MX_TIM3_Encoder_Init() {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  // D11 = PB4 (TIM3_CH1), D12 = PB5 (TIM3_CH2)
  GPIO_InitTypeDef GPIO_InitStruct{};
  GPIO_InitStruct.Pin       = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig{};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 2;
  sConfig.IC2Polarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 2;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 0xFFFFFFFFUL;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.RepetitionCounter = 0;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  __HAL_TIM_SET_COUNTER(&htim3, ENC_MID);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

// Appliquer commande PWM moteur
void setMotorPWM(float cmd) {
  // Saturer à ±255
  if (cmd > 255.0f) cmd = 255.0f;
  if (cmd < -255.0f) cmd = -255.0f;
  
  if (cmd >= 0) {
    analogWrite(M_PWM_P, (int)cmd);
    analogWrite(M_PWM_N, 0);
  } else {
    analogWrite(M_PWM_P, 0);
    analogWrite(M_PWM_N, (int)(-cmd));
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(M_PWM_P, OUTPUT);
  pinMode(M_PWM_N, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  MX_TIM2_Encoder_Init();
  MX_TIM3_Encoder_Init();
  
  delay(1000);
  
  Serial.println("=== RÉGULATION PID MOTEUR ===");
  Serial.println("Moteur: D9(+) / D3(-)");
  Serial.println("Encodeur droit: A0/A1 (TIM2)");
  Serial.println("Encodeur gauche: D11/D12 (TIM3)");
  Serial.println("Consignes automatiques...");
  Serial.println("==============================");
  
  setMotorPWM(0); // Moteur arrêté
}

void loop() {
  static unsigned long lastPidTime = 0;
  static unsigned long lastDisplayTime = 0;
  static bool ledState = false;
  static uint8_t targetPhase = 0;
  
  // Lecture encodeur droit
  uint32_t rawCount = __HAL_TIM_GET_COUNTER(&htim2);
  encoderCount = (int32_t)rawCount - (int32_t)ENC_MID;
  
  // Lecture encodeur gauche
  uint32_t rawCountLeft = __HAL_TIM_GET_COUNTER(&htim3);
  encoderCountLeft = (int32_t)rawCountLeft - (int32_t)ENC_MID;
  
  // Calcul PID toutes les 50ms
  if (millis() - lastPidTime >= (unsigned long)(DT_S * 1000)) {
    
    // Calculer vitesse actuelle moteur droit en tr/s
    int32_t deltaEnc = encoderCount - lastEncoderCount;
    lastEncoderCount = encoderCount;
    currentSpeed_rps = (float)deltaEnc / TICKS_PER_REV / DT_S;
    
    // Calculer vitesse actuelle moteur gauche en tr/s
    int32_t deltaEncLeft = encoderCountLeft - lastEncoderCountLeft;
    lastEncoderCountLeft = encoderCountLeft;
    currentSpeedLeft_rps = (float)deltaEncLeft / TICKS_PER_REV / DT_S;
    
    // Calcul PID
    error = targetSpeed_rps - currentSpeed_rps;
    errorSum += error * DT_S;
    float errorDiff = (error - errorPrev) / DT_S;
    
    pidOutput = kp * error + ki * errorSum + kd * errorDiff;
    errorPrev = error;
    
    // Saturation anti-windup simple
    if (errorSum > 100.0f) errorSum = 100.0f;
    if (errorSum < -100.0f) errorSum = -100.0f;
    
    // Appliquer au moteur
    setMotorPWM(pidOutput);
    
    lastPidTime = millis();
  }
  
  // Affichage + changement consigne toutes les 200ms
  if (millis() - lastDisplayTime >= 200) {
    compteur++;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH);
    
    // Changer consigne toutes les 25 fois (5 secondes)
    if (compteur % 25 == 0) {
      targetPhase = (targetPhase + 1) % 5;
      
      switch(targetPhase) {
        case 0: targetSpeed_rps = 0.0f; Serial.println(">> CONSIGNE: 0 tr/s (ARRÊT)"); break;
        case 1: targetSpeed_rps = 30.0f; Serial.println(">> CONSIGNE: 1 tr/s (LENT +)"); break;
        case 2: targetSpeed_rps = 0.0f; Serial.println(">> CONSIGNE: -1 tr/s (LENT -)"); break;
        case 3: targetSpeed_rps = -30.0f; Serial.println(">> CONSIGNE: 2 tr/s (RAPIDE +)"); break;
        case 4: targetSpeed_rps = -50.0f; Serial.println(">> CONSIGNE: -2 tr/s (RAPIDE -)"); break;
      }
      // Reset intégrateur à chaque changement
      errorSum = 0.0f;
    }
    
    // Affichage PID
    Serial.print("Consigne: ");
    Serial.print(targetSpeed_rps, 1);
    Serial.print(" | Mesure D: ");
    Serial.print(currentSpeed_rps, 2);
    Serial.print(" | Mesure G: ");
    Serial.print(currentSpeedLeft_rps, 2);
    Serial.print(" | Erreur: ");
    Serial.print(error, 2);
    Serial.print(" | PWM: ");
    Serial.print(pidOutput, 0);
    Serial.print(" | Enc D: ");
    Serial.print(encoderCount);
    Serial.print(" | Enc G: ");
    Serial.println(encoderCountLeft);
    
    lastDisplayTime = millis();
  }
  
  delay(10);
}
