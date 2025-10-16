/* NUCLEO-G431KB - RÉGULATION PID DOUBLE MOTEUR
 * 
 * MOTEUR DROIT (code existant conservé):
 *   - PWM: D9=PA8 (+), D3=PB0 (-)
 *   - Encodeur: A0=PA0 (TIM2_CH1), A1=PA1 (TIM2_CH2)
 * 
 * MOTEUR GAUCHE (ajout):
 *   - PWM: D2=PA10 (+), D10=PA11 (-)
 *   - Encodeur: A6=PB4 (TIM3_CH1), A5=PB5 (TIM3_CH2)
 */

#include <Arduino.h>
extern "C" {
  #include "stm32g4xx_hal.h"
}

// Pins PWM moteur DROIT (existant)
const uint8_t M_PWM_P = D9;   // TIM1_CH1  (PA8)  - Moteur +
const uint8_t M_PWM_N = D3;   // TIM1_CH2N (PB0)  - Moteur -

// ==================== AJOUT MOTEUR GAUCHE ====================
const uint8_t ML_PWM_P = D2;   // TIM1_CH3  (PA12) - Moteur gauche +
const uint8_t ML_PWM_N = D10;  // TIM1_CH4  (PA11) - Moteur gauche -

// Encodeur TIM2 (existant)
TIM_HandleTypeDef htim2;
static constexpr uint32_t ENC_MID = 0x7FFFFFFFUL;

// ==================== AJOUT ENCODEUR GAUCHE ====================
TIM_HandleTypeDef htim3;

// Paramètres encodeur (existant)
static constexpr float CPR_MOTOR = 1200.0f;
static constexpr float GEAR_RATIO = 100.0f;
static constexpr float QUAD_FACTOR = 4.0f;
static constexpr float TICKS_PER_REV = (CPR_MOTOR * QUAD_FACTOR) / GEAR_RATIO;

// Paramètres PID (existant)
static constexpr float DT_S = 0.050f;

// Gains PID (existant)
float kp = 0.8f;
float ki = 0.2f;
float kd = 0.1f;

// Variables PID moteur DROIT (existant)
float targetSpeed_rps = 0.0f;
float currentSpeed_rps = 0.0f;
float error = 0.0f, errorPrev = 0.0f, errorSum = 0.0f;
float pidOutput = 0.0f;
int32_t encoderCount = 0, lastEncoderCount = 0;

// ==================== AJOUT VARIABLES MOTEUR GAUCHE ====================
float targetSpeed_rps_L = 0.0f;
float currentSpeed_rps_L = 0.0f;
float error_L = 0.0f, errorPrev_L = 0.0f, errorSum_L = 0.0f;
float pidOutput_L = 0.0f;
int32_t encoderCount_L = 0, lastEncoderCount_L = 0;

unsigned long compteur = 0;

// Configuration encodeur TIM2 (EXISTANT - NON MODIFIÉ)
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

// ==================== AJOUT ENCODEUR TIM3 ====================
void MX_TIM3_Encoder_Init() {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  // PB4 (A6) = TIM3_CH1, PB5 (A5) = TIM3_CH2
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
  htim3.Init.Period            = 0xFFFF;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.RepetitionCounter = 0;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  __HAL_TIM_SET_COUNTER(&htim3, 0x8000);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

// Appliquer commande PWM moteur DROIT (EXISTANT - NON MODIFIÉ)
void setMotorPWM(float cmd) {
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

// ==================== AJOUT COMMANDE MOTEUR GAUCHE ====================
void setMotorPWM_Left(float cmd) {
  if (cmd > 255.0f) cmd = 255.0f;
  if (cmd < -255.0f) cmd = -255.0f;
  
  if (cmd >= 0) {
    analogWrite(ML_PWM_P, (int)cmd);
    analogWrite(ML_PWM_N, 0);
  } else {
    analogWrite(ML_PWM_P, 0);
    analogWrite(ML_PWM_N, (int)(-cmd));
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(M_PWM_P, OUTPUT);
  pinMode(M_PWM_N, OUTPUT);
  pinMode(ML_PWM_P, OUTPUT);
  pinMode(ML_PWM_N, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  MX_TIM2_Encoder_Init();
  MX_TIM3_Encoder_Init();      // MODIFIÉ: TIM3 au lieu de TIM15
  
  delay(1000);
  
  Serial.println("=== RÉGULATION PID DOUBLE MOTEUR ===");
  Serial.println("DROIT: D9/D3 + A0/A1 (TIM2)");
  Serial.println("GAUCHE: D2/D10 + A6/A5 (TIM3)");
  Serial.println("=====================================");
  
  setMotorPWM(0);
  setMotorPWM_Left(0);
}

void loop() {
  static unsigned long lastPidTime = 0;
  static unsigned long lastDisplayTime = 0;
  static bool ledState = false;
  static uint8_t targetPhase = 0;
  
  // Lecture encodeur DROIT (EXISTANT)
  uint32_t rawCount = __HAL_TIM_GET_COUNTER(&htim2);
  encoderCount = (int32_t)rawCount - (int32_t)ENC_MID;
  
  // ==================== AJOUT LECTURE ENCODEUR GAUCHE ====================
  uint16_t rawCount_L = __HAL_TIM_GET_COUNTER(&htim3);
  encoderCount_L = (int32_t)rawCount_L - 0x8000;
  
  // Calcul PID toutes les 50ms
  if (millis() - lastPidTime >= (unsigned long)(DT_S * 1000)) {
    
    // -------- PID MOTEUR DROIT (EXISTANT - NON MODIFIÉ) --------
    int32_t deltaEnc = encoderCount - lastEncoderCount;
    lastEncoderCount = encoderCount;
    currentSpeed_rps = (float)deltaEnc / TICKS_PER_REV / DT_S;
    
    error = targetSpeed_rps - currentSpeed_rps;
    errorSum += error * DT_S;
    float errorDiff = (error - errorPrev) / DT_S;
    
    pidOutput = kp * error + ki * errorSum + kd * errorDiff;
    errorPrev = error;
    
    if (errorSum > 100.0f) errorSum = 100.0f;
    if (errorSum < -100.0f) errorSum = -100.0f;
    
    setMotorPWM(pidOutput);
    
    // ==================== AJOUT PID MOTEUR GAUCHE ====================
    int32_t deltaEnc_L = encoderCount_L - lastEncoderCount_L;
    lastEncoderCount_L = encoderCount_L;
    currentSpeed_rps_L = (float)deltaEnc_L / TICKS_PER_REV / DT_S;
    
    error_L = targetSpeed_rps_L - currentSpeed_rps_L;
    errorSum_L += error_L * DT_S;
    float errorDiff_L = (error_L - errorPrev_L) / DT_S;
    
    pidOutput_L = kp * error_L + ki * errorSum_L + kd * errorDiff_L;
    errorPrev_L = error_L;
    
    if (errorSum_L > 100.0f) errorSum_L = 100.0f;
    if (errorSum_L < -100.0f) errorSum_L = -100.0f;
    
    setMotorPWM_Left(pidOutput_L);
    
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
        case 0: 
          targetSpeed_rps = 0.0f;
          targetSpeed_rps_L = 0.0f;
          Serial.println(">> CONSIGNE: 0 tr/s (ARRÊT)"); 
          break;
        case 1: 
          targetSpeed_rps = 30.0f;
          targetSpeed_rps_L = 30.0f;
          Serial.println(">> CONSIGNE: 30 tr/s (AVANCE)"); 
          break;
        case 2: 
          targetSpeed_rps = 0.0f;
          targetSpeed_rps_L = 0.0f;
          Serial.println(">> CONSIGNE: 0 tr/s (RECUL)"); 
          break;
        case 3: 
          targetSpeed_rps = 30.0f;
          targetSpeed_rps_L = -30.0f;
          Serial.println(">> CONSIGNE: 30/-30 tr/s (ROTATION)"); 
          break;
        case 4: 
          targetSpeed_rps = 50.0f;
          targetSpeed_rps_L = 50.0f;
          Serial.println(">> CONSIGNE: 50 tr/s (RAPIDE)"); 
          break;
      }
      errorSum = 0.0f;
      errorSum_L = 0.0f;
    }
    
    // Affichage PID
    Serial.print("D-Cons:");
    Serial.print(targetSpeed_rps, 1);
    Serial.print(" Mes:");
    Serial.print(currentSpeed_rps, 2);
    Serial.print(" PWM:");
    Serial.print(pidOutput, 0);
    Serial.print(" | G-Cons:");
    Serial.print(targetSpeed_rps_L, 1);
    Serial.print(" Mes:");
    Serial.print(currentSpeed_rps_L, 2);
    Serial.print(" PWM:");
    Serial.println(pidOutput_L, 0);
    
    lastDisplayTime = millis();
  }
  
  delay(10);
}
