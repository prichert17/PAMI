/*  NUCLEO-L432KC – 2 MCC avec encodeurs HW + PID en timer
 *  PWM: TIM1  (4 canaux sur D9/D10/D3/D6)
 *  ENC1: TIM2 (PA0=A0 CH1, PA1=A1 CH2)
 *  ENC2: TIM15(PA2=A7 CH1, PA3=A2 CH2)  -> Couper SB2 (VCP_TX) si utilisé
 *  PID: TIM6  (10 ms)
 *
 *  Mapping récap:
 *    Moteur Droit  : PWM+ -> D9  (PA8  TIM1_CH1)
 *                    PWM- -> D3  (PB0  TIM1_CH2N)
 *    Moteur Gauche : PWM+ -> D10 (PA11 TIM1_CH4)
 *                    PWM- -> D6  (PB1  TIM1_CH3N)
 *    Encodeur Droit: A0=PA0 (TIM2_CH1), A1=PA1 (TIM2_CH2)
 *    Encodeur Gauche: A7=PA2 (TIM15_CH1), A2=PA3 (TIM15_CH2)  [SB2 OFF]
 *    Série debug   : Serial1 (USART1) sur D1=PA9 TX / D0=PA10 RX
 *
 *  Hypothèses encodeur:
 *    1400 impulsions/tour moteur, réduction 100:1 -> 14 impulsions/tour roue.
 *    Timer en mode X4 => facteur_quadrature = 4 -> 56 ticks / tour roue.
 *    Ajuster si votre codeur diffère.
 */

#include <Arduino.h>
extern "C" {
  #include "stm32l4xx_hal.h"
}

// ------------ Paramètres mécaniques / encodeurs ------------
static constexpr float CPR_MOTOR   = 1400.0f;  // impulsions par tour moteur
static constexpr float GEAR_RATIO  = 100.0f;   // réducteur 100:1
static constexpr int   QUAD_FACTOR = 4;        // mode encodeur X4
static constexpr float TICKS_PER_WHEEL_REV = (CPR_MOTOR / GEAR_RATIO) * QUAD_FACTOR; // ~56

// ------------ PID ------------
static constexpr float DT_PID_S   = 0.010f; // 10 ms
static constexpr uint32_t PID_US  = 10000;  // 10 000 µs

// Gains à ajuster
float kp = 0.5f, ki = 0.3f, kd = -0.2f;

// Cible "style ancien code": 0..255
volatile float targetCmdRight = 255.0f;
volatile float targetCmdLeft  = 255.0f;

// Conversion vitesse roue [tr/s] -> "échelle PWM" (à calibrer)
static constexpr float KSPEED = 227.0f;

// ------------ Pins PWM (TIM1) ------------
const uint8_t M1_PWM_P = D9;   // TIM1_CH1  (PA8)
const uint8_t M1_PWM_N = D3;   // TIM1_CH2N (PB0)
const uint8_t M2_PWM_P = D10;  // TIM1_CH4  (PA11)
const uint8_t M2_PWM_N = D6;   // TIM1_CH3N (PB1)

// ------------ Pins encodeurs ------------
const uint8_t ENC1_A = A0; // PA0 TIM2_CH1
const uint8_t ENC1_B = A1; // PA1 TIM2_CH2
const uint8_t ENC2_A = A7; // PA2 TIM15_CH1  [SB2 OFF]
const uint8_t ENC2_B = A2; // PA3 TIM15_CH2

// ------------ Timers HW ------------
HardwareTimer *pwmTim = nullptr;     // TIM1
HardwareTimer *pidTim = nullptr;     // TIM6

// HAL handles pour encodeurs
TIM_HandleTypeDef htim2;   // 32 bits
TIM_HandleTypeDef htim15;  // 16 bits

// Compteurs centrés pour deltas signés
static constexpr uint32_t ENC2_MID = 0x7FFFFFFFUL; // TIM2
static constexpr uint16_t ENC15_MID= 0x7FFF;       // TIM15

// Variables PID
volatile float speedR_rps = 0.0f, speedL_rps = 0.0f;
volatile float eR=0, eL=0, eR_prev=0, eL_prev=0, iR=0, iL=0;

// ----------- PWM helpers (TIM1) -----------
static constexpr uint32_t PWM_FREQ_HZ = 20000;  // 20 kHz inaudible

// canaux TIM1 correspondant aux pins ci-dessus
static constexpr uint32_t CH_M1P = 1; // PA8  -> CH1
static constexpr uint32_t CH_M1N = 2; // PB0  -> CH2N (donner CH2)
static constexpr uint32_t CH_M2P = 4; // PA11 -> CH4
static constexpr uint32_t CH_M2N = 3; // PB1  -> CH3N (donner CH3)

// fixe rapport cyclique 0..255 sur un canal TIM1
inline void setDutyTIM1(uint32_t ch, uint8_t duty8) {
  pwmTim->setCaptureCompare(ch, duty8, RESOLUTION_8B_COMPARE_FORMAT);
}

// applique commande signée sur un pont en H (2 PWM)
void set_vitesse(float cmdRight, float cmdLeft) {
  // Saturation
  if (cmdRight > 255.0f) cmdRight = 255.0f;
  if (cmdRight < -255.0f) cmdRight = -255.0f;
  if (cmdLeft  > 255.0f) cmdLeft  = 255.0f;
  if (cmdLeft  < -255.0f) cmdLeft  = -255.0f;

  // Moteur droit: M1_PWM_P / M1_PWM_N
  if (cmdRight >= 0) {
    setDutyTIM1(CH_M1P, (uint8_t)cmdRight);
    setDutyTIM1(CH_M1N, 0);
  } else {
    setDutyTIM1(CH_M1P, 0);
    setDutyTIM1(CH_M1N, (uint8_t)(-cmdRight));
  }

  // Moteur gauche: M2_PWM_P / M2_PWM_N
  if (cmdLeft >= 0) {
    setDutyTIM1(CH_M2P, (uint8_t)cmdLeft);
    setDutyTIM1(CH_M2N, 0);
  } else {
    setDutyTIM1(CH_M2P, 0);
    setDutyTIM1(CH_M2N, (uint8_t)(-cmdLeft));
  }
}

// ----------- HAL init encodeurs -----------
void MX_TIM2_Encoder_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  // PA0 (CH1), PA1 (CH2) AF1
  GPIO_InitTypeDef GPIO_InitStruct{};
  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig{};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // X4
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
  htim2.Init.Period            = 0xFFFFFFFFUL;  // 32 bits
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.RepetitionCounter = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim2, &sConfig);
  __HAL_TIM_SET_COUNTER(&htim2, ENC2_MID);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void MX_TIM15_Encoder_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM15_CLK_ENABLE();

  // PA2 (CH1), PA3 (CH2) AF14
  GPIO_InitTypeDef GPIO_InitStruct{};
  GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig{};
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // X4
  sConfig.IC1Polarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 2;
  sConfig.IC2Polarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 2;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler         = 0;
  htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim15.Init.Period            = 0xFFFF;  // 16 bits
  htim15.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Encoder_Init(&htim15, &sConfig);
  __HAL_TIM_SET_COUNTER(&htim15, ENC15_MID);
  HAL_TIM_Encoder_Start(&htim15, TIM_CHANNEL_ALL);
}

// ----------- PID ISR (10 ms) -----------
void onPidTick() {
  // Lire delta ticks depuis centre
  int32_t cnt2  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t cnt15 = (int16_t)__HAL_TIM_GET_COUNTER(&htim15);

  int32_t dR = cnt2  - (int32_t)ENC2_MID;   // droit
  int32_t dL = (int32_t)cnt15 - (int32_t)ENC15_MID; // gauche

  __HAL_TIM_SET_COUNTER(&htim2,  ENC2_MID);
  __HAL_TIM_SET_COUNTER(&htim15, ENC15_MID);

  // Ticks -> tr/s roue
  speedR_rps = (float)dR / TICKS_PER_WHEEL_REV / DT_PID_S;
  speedL_rps = (float)dL / TICKS_PER_WHEEL_REV / DT_PID_S;

  // Mesure ramenée à échelle "commande 0..255"
  float measR = KSPEED * speedR_rps;
  float measL = KSPEED * speedL_rps;

  // PID
  eR = targetCmdRight - measR;
  eL = targetCmdLeft  - measL;

  iR += eR * DT_PID_S;
  iL += eL * DT_PID_S;

  float dEdtR = (eR - eR_prev) / DT_PID_S;
  float dEdtL = (eL - eL_prev) / DT_PID_S;

  float cmdR = kp*eR + ki*iR + kd*dEdtR;
  float cmdL = kp*eL + ki*iL + kd*dEdtL;

  eR_prev = eR; eL_prev = eL;

  set_vitesse(cmdR, cmdL);

  // Log série: vitesse et commande
  Serial.print("vR=");
  Serial.print(speedR_rps, 3);
  Serial.print(" cmdR=");
  Serial.print(cmdR, 1);
  Serial.print(" | vL=");
  Serial.print(speedL_rps, 3);
  Serial.print(" cmdL=");
  Serial.println(cmdL, 1);
}

void setup() {
  // Série debug sur USB au lieu de USART1
  Serial.begin(115200);

  // --- PWM TIM1 ---
  pwmTim = new HardwareTimer(TIM1);
  // Configure chaque canal une fois; duty initial 0%
  pwmTim->setPWM(CH_M1P, M1_PWM_P, PWM_FREQ_HZ, 0);
  pwmTim->setPWM(CH_M1N, M1_PWM_N, PWM_FREQ_HZ, 0);
  pwmTim->setPWM(CH_M2P, M2_PWM_P, PWM_FREQ_HZ, 0);
  pwmTim->setPWM(CH_M2N, M2_PWM_N, PWM_FREQ_HZ, 0);

  // --- Encodeurs TIM2 / TIM15 ---
  MX_TIM2_Encoder_Init();
  MX_TIM15_Encoder_Init();

  // --- PID timer TIM6 @10ms ---
  pidTim = new HardwareTimer(TIM6);
  pidTim->setOverflow(PID_US, MICROSEC_FORMAT);
  pidTim->attachInterrupt(onPidTick);
  pidTim->resume();

  // Démarrage: applique la cible initiale
  set_vitesse(targetCmdRight, targetCmdLeft);
}

void loop() {
  // Exemple: cible fixe. Expose une API similaire à l'ancien code.
  // Adapter en fonction de vos besoins.
  // set_vitesse(targetCmdRight, targetCmdLeft); // la boucle PID pilote déjà
}
