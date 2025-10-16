/* NUCLEO-G431KB - REGULATION PID DOUBLE MOTEUR (SOLUTION HYBRIDE FINALE)
 *
 * MOTEUR DROIT: (Piloté par la HAL pour une stabilité maximale)
 * - PWM: D9=PA8 (+), D3=PB0 (-)        -> TIM1 configuré manuellement
 * - Encodeur: A0=PA0, A1=PA1          -> TIM2
 *
 * MOTEUR GAUCHE: (Piloté par Arduino)
 * - PWM: D2=PA12 (+), A2=PA3 (-)        -> TIM16 & TIM15 via analogWrite
 * - Encodeur: PB4, PB5                  -> TIM3
 */

#include <Arduino.h>
extern "C" {
  #include "stm32g4xx_hal.h"
}

// ===== RESSOURCES MOTEUR DROIT =====
TIM_HandleTypeDef htim1; // Pour le PWM HAL
TIM_HandleTypeDef htim2; // Pour l'encodeur
const uint8_t M_PWM_P_PIN = D9;
const uint8_t M_PWM_N_PIN = D3;
static constexpr uint32_t ENC_MID = 0x7FFFFFFFUL;

// ===== RESSOURCES MOTEUR GAUCHE =====
TIM_HandleTypeDef htim3; // Pour l'encodeur
const uint8_t ML_PWM_P_PIN = D2;
const uint8_t ML_PWM_N_PIN = A2;
static constexpr uint16_t ENC_MID_16 = 0x8000U;


// ===== VARIABLES PID & ENCODEURS =====
// ... (Toutes les variables pour les deux moteurs, inchangées)
float targetSpeed_rps = 0.0f,    targetSpeed_rps_L = 0.0f;
float currentSpeed_rps = 0.0f,   currentSpeed_rps_L = 0.0f;
float error = 0.0f,              error_L = 0.0f;
float errorPrev = 0.0f,          errorPrev_L = 0.0f;
float errorSum = 0.0f,           errorSum_L = 0.0f;
float pidOutput = 0.0f,          pidOutput_L = 0.0f;
int32_t encoderCount = 0,        encoderCount_L = 0;
int32_t lastEncoderCount = 0,    lastEncoderCount_L = 0;
uint32_t encoderOffset = 0;  // Référence initiale pour encodeur droit


// ===== PARAMÈTRES COMMUNS =====
static constexpr float CPR_MOTOR = 1200.0f;
static constexpr float GEAR_RATIO = 100.0f;
static constexpr float QUAD_FACTOR = 4.0f;
static constexpr float TICKS_PER_REV = (CPR_MOTOR * QUAD_FACTOR) / GEAR_RATIO;
static constexpr float DT_S = 0.050f;
float kp = 0.8f, ki = 0.2f, kd = 0.1f;
unsigned long compteur = 0;
int targetPhase = 0; // phase de consigne pour les séquences (0..4)


// ===============================================================
//           INITIALISATION DES PÉRIPHÉRIQUES (HAL)
// ===============================================================

// PWM MOTEUR DROIT (via HAL sur TIM1)
void MX_TIM1_PWM_Droit_Init() {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;

  GPIO_InitStruct.Pin = GPIO_PIN_8; // PA8 = D9
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_0; // PB0 = D3
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8499; // Pour 20kHz
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim1);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_MOE_ENABLE(&htim1); // Activation de la sortie principale du timer avancé
}

// ENCODEUR MOTEUR DROIT (TIM2)
void MX_TIM2_Encoder_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct{};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1; // A0, A1
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  TIM_Encoder_InitTypeDef sConfig{};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFFUL;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Encoder_Init(&htim2, &sConfig);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, ENC_MID);  // Réinitialisation APRÈS le démarrage
}

// ENCODEUR MOTEUR GAUCHE (TIM3)
void MX_TIM3_Encoder_Init() {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct{};
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5; // PB4, PB5
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  TIM_Encoder_InitTypeDef sConfig{};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFFU; // Timer 16 bits
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, ENC_MID_16);  // Réinitialisation APRÈS le démarrage
}

// ===============================================================
//                     COMMANDES MOTEURS
// ===============================================================

// Moteur Droit via HAL
void setMotorPWM_Droit_HAL(float cmd) {
  if (cmd > 255.0f) cmd = 255.0f;
  if (cmd < -255.0f) cmd = -255.0f;
  uint32_t pulse = (uint32_t)((fabsf(cmd) / 255.0f) * 8499);
  if (cmd >= 0) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
  }
}

// Moteur Gauche via Arduino
void setMotorPWM_Gauche_Arduino(float cmd) {
  if (cmd > 255.0f) cmd = 255.0f;
  if (cmd < -255.0f) cmd = -255.0f;
  if (cmd >= 0) {
    analogWrite(ML_PWM_P_PIN, (int)cmd);
    analogWrite(ML_PWM_N_PIN, 0);
  } else {
    analogWrite(ML_PWM_P_PIN, 0);
    analogWrite(ML_PWM_N_PIN, (int)(-cmd));
  }
}

// ===============================================================
//                       PROGRAMME PRINCIPAL
// ===============================================================

void setup() {
  Serial.begin(115200);
  
  // Configuration PWM Moteur Gauche (simple)
  pinMode(ML_PWM_P_PIN, OUTPUT);
  pinMode(ML_PWM_N_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialisation des périphériques HAL
  MX_TIM1_PWM_Droit_Init();
  MX_TIM2_Encoder_Init();
  MX_TIM3_Encoder_Init();
  
  // Sauvegarder la valeur initiale de l'encodeur droit comme référence
  encoderOffset = __HAL_TIM_GET_COUNTER(&htim2);
  
  delay(1000);
  Serial.println("=== SOLUTION HYBRIDE HAL + ARDUINO ===");
  
  setMotorPWM_Droit_HAL(0);
  setMotorPWM_Gauche_Arduino(0);
}

void loop() {
    static unsigned long lastPidTime = 0;
    static unsigned long lastDisplayTime = 0;

    // --- Lecture des encodeurs ---
    uint32_t rawCount = __HAL_TIM_GET_COUNTER(&htim2);
    encoderCount = (int32_t)(rawCount - encoderOffset);
    
    // Encodeur gauche avec gestion overflow 16 bits
    uint16_t rawCount_L = __HAL_TIM_GET_COUNTER(&htim3);
    int16_t delta_raw = (int16_t)(rawCount_L - (uint16_t)ENC_MID_16);
    encoderCount_L += delta_raw;  // Accumulation pour éviter l'overflow
    __HAL_TIM_SET_COUNTER(&htim3, ENC_MID_16);  // Reset pour prochaine lecture

    // --- Boucle PID (toutes les 50ms) ---
    if (millis() - lastPidTime >= (unsigned long)(DT_S * 1000)) {
        // --- PID Moteur Droit ---
        int32_t deltaEnc = encoderCount - lastEncoderCount;
        lastEncoderCount = encoderCount;
        currentSpeed_rps = (float)deltaEnc / TICKS_PER_REV / DT_S;
        error = targetSpeed_rps - currentSpeed_rps;
        errorSum += error * DT_S;
        if (errorSum > 100.0f) errorSum = 100.0f; if (errorSum < -100.0f) errorSum = -100.0f;
        pidOutput = kp * error + ki * errorSum + kd * (error - errorPrev);
        errorPrev = error;
        setMotorPWM_Droit_HAL(pidOutput);

        // --- PID Moteur Gauche ---
        int32_t deltaEnc_L = encoderCount_L - lastEncoderCount_L;
        lastEncoderCount_L = encoderCount_L;
        currentSpeed_rps_L = (float)deltaEnc_L / TICKS_PER_REV / DT_S;
        error_L = targetSpeed_rps_L - currentSpeed_rps_L;
        errorSum_L += error_L * DT_S;
        if (errorSum_L > 100.0f) errorSum_L = 100.0f; if (errorSum_L < -100.0f) errorSum_L = -100.0f;
        pidOutput_L = kp * error_L + ki * errorSum_L + kd * (error_L - errorPrev_L);
        errorPrev_L = error_L;
        setMotorPWM_Gauche_Arduino(pidOutput_L);

        lastPidTime = millis();
    }

    // --- Affichage et changement de consigne ---
    if (millis() - lastDisplayTime >= 200) {
        compteur++;
        if (compteur % 25 == 0) { // Toutes les 5 secondes
            targetPhase = (targetPhase + 1) % 5;
            switch(targetPhase) {
                case 0: targetSpeed_rps = 0.0f;   targetSpeed_rps_L = 0.0f;   Serial.println(">> C: ARRET"); break;
                case 1: targetSpeed_rps = 30.0f;  targetSpeed_rps_L = 30.0f;  Serial.println(">> C: AVANT"); break;
                case 2: targetSpeed_rps = -30.0f; targetSpeed_rps_L = -30.0f; Serial.println(">> C: ARRIERE"); break;
                case 3: targetSpeed_rps = 30.0f;  targetSpeed_rps_L = -30.0f; Serial.println(">> C: TOURNER"); break;
                case 4: targetSpeed_rps = 50.0f;  targetSpeed_rps_L = 50.0f;  Serial.println(">> C: VITE"); break;
            }
            errorSum = 0.0f; errorSum_L = 0.0f;
        }
        Serial.print("D: M="); Serial.print(currentSpeed_rps, 1); Serial.print(",P="); Serial.print(pidOutput, 0);
        Serial.print(" | G: M="); Serial.print(currentSpeed_rps_L, 1); Serial.print(",P="); Serial.print(pidOutput_L, 0);
        Serial.print(" | Enc: D="); Serial.print(encoderCount); Serial.print(",G="); Serial.println(encoderCount_L);
        lastDisplayTime = millis();
    }
    delay(10);
}