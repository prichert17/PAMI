#include "main.hpp"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "control/motor.hpp"
#include "control/odometry.hpp"
#include "control/asservissement.hpp"
#include "utils/printf.hpp"
#include <cstring>
#include <array>

void SystemClock_Config(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// Instance de SerialOut pour la communication UART
SerialOut serial;

// Variables globales
uint8_t rxByte;
char cmd_buffer[20] = {0};
uint8_t cmd_idx = 0;

// Variables ADC
uint32_t adc_values;  // variable pour stocker les valeurs ADC

// Seuils de tension pour LEDs batterie (après pont diviseur /2)
// LiFePO4 1S: pleine charge ~3.6V -> 1.8V, déchargée ~2.5V -> 1.25V
// USB 5V -> 2.5V
#define BATTERY_FULL_THRESHOLD     1.6f   // Tension seuil batterie bien chargée (V après diviseur) = 3.2V réel
#define BATTERY_LOW_THRESHOLD      1.45f  // Tension seuil batterie faible (V après diviseur) = 2.9V réel
#define BATTERY_CRITICAL_THRESHOLD 1.35f  // Tension critique (V après diviseur) = 2.7V réel
#define USB_VOLTAGE_MAX            2.55f  // Tension USB max (5V / 2 = 2.5V)
#define USB_VOLTAGE_MIN            2.4f   // Tension USB min

// Fonction pour mettre à jour les LEDs de batterie
void update_battery_leds(float voltage) {
    static uint32_t blink_timer = 0;
    static bool blink_state = false;
    
    // Si tension USB (~2.5V après diviseur) -> les deux LEDs allumées
    if (voltage >= USB_VOLTAGE_MIN && voltage <= USB_VOLTAGE_MAX) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  // LED verte OFF
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  // LED rouge OFF
    }
    // Batterie dangereusement basse -> LED rouge clignotante
    else if (voltage < BATTERY_CRITICAL_THRESHOLD) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // LED verte OFF
        // Clignotement toutes les 250ms
        if (HAL_GetTick() - blink_timer >= 250) {
            blink_state = !blink_state;
            blink_timer = HAL_GetTick();
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, blink_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    // Batterie faible (entre critical et low) -> LED rouge seule
    else if (voltage < BATTERY_LOW_THRESHOLD) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // LED verte OFF
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // LED rouge ON
    }
    // Batterie moyenne (entre low et full) -> les deux LEDs
    else if (voltage < BATTERY_FULL_THRESHOLD) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // LED verte ON
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // LED rouge ON
    }
    // Batterie bien chargée -> LED verte seule
    else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // LED verte ON
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // LED rouge OFF
    }
}

// Mode de fonctionnement : true = test manuel, false = asservi
bool test_mode = true;

// Moteurs utilisant la classe Wheel
std::array<Wheel, 3> wheels = {
    Wheel(1, 1200, 29.0f, 0.0f),     // Moteur 1
    Wheel(2, 1200, 29.0f, 180.0f),   // Moteur 2
    Wheel(3, 1200, 29.0f, 0.0f)      // Moteur 3 (réserve)
};

// Odométrie et asservissement
Odometry odometry(wheels);
Asserv_Position asserv(&odometry, &wheels);

// Variables pour la position cible
float target_x = 0.0f;
float target_y = 0.0f;
float target_z = 0.0f;  // Rotation en degrés

// Compteur pour l'affichage périodique
static uint32_t lastSend = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialisation des périphériques
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_TIM15_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    // Calibration ADC - IMPORTANT: avant toute utilisation
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        serial.send("Erreur calibration ADC1\r\n");
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        serial.send("Erreur calibration ADC2\r\n");
    }

    // Initialisation des encodeurs au milieu (32768)
    TIM2->CNT = (1<<15);
    TIM3->CNT = (1<<15);

    // Démarrage des encodeurs
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    // Démarrage des PWM moteurs avec canaux complémentaires
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // Moteur 1 CH1
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);    // Moteur 1 CH1N
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);      // Moteur 2 CH1
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);   // Moteur 2 CH1N

    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim16);
    //__HAL_TIM_MOE_ENABLE(&htim8);

    // Initialiser les moteurs à 0
    wheels[0].set_pwm(0);
    wheels[1].set_pwm(0);

    // Démarrage réception UART
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);

    // Démarrage des timers pour interruptions
    // TIM6 : Odométrie 500Hz
    // TIM15 : Timer générique 1Hz
    // TIM17 : Asservissement 100Hz
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim15);
    HAL_TIM_Base_Start_IT(&htim17);

    // Configuration PID pour l'asservissement en position
    asserv.set_PID(1.0, 1.0, 0.0);

    // Message de démarrage
    serial.send("\r\n=== PAMI STM32 ===\r\n");
    serial.send("mode auto   -> Mode automatique\r\n");
    serial.send("mode manuel -> Mode manuel\r\n");
    serial.send("AUTO: X:xxx Y:xxx (mm)\r\n");
    serial.send("MANUEL: M1:xxx M2:xxx (-1000/+1000), on, off, stop\r\n");
    serial.send("==================\r\n\r\n");

    while (1) {
        // Affichage toutes les 500ms (dans les deux modes)
        if (HAL_GetTick() - lastSend > 500) {
            // Mise à jour des vitesses
            wheels[0].update_speed(0.5f);
            wheels[1].update_speed(0.5f);
            
            // Lecture des encodeurs et vitesses
            int32_t enc1 = wheels[0].get_encoder_count();
            int32_t enc2 = wheels[1].get_encoder_count();
            float speed1 = wheels[0].get_speed();
            float speed2 = wheels[1].get_speed();
            
            // Conversion rad/s -> ticks/s
            int32_t speed1_ticks = (int32_t)(speed1 * 1200 / (2.0f * M_PI));
            int32_t speed2_ticks = (int32_t)(speed2 * 1200 / (2.0f * M_PI));
            
            // Lecture ADC1 (PB0 - Canal 15)
            adc_values = 0;
            if (HAL_ADC_Start(&hadc1) == HAL_OK) {
                if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                    adc_values = HAL_ADC_GetValue(&hadc1);
                }
                HAL_ADC_Stop(&hadc1);
            }
            
            // Conversion en tension (3.3V de référence, résolution 12 bits = 4095)
            float voltage1 = (adc_values * 3.3f) / 4095.0f;
            
            // Mise à jour des LEDs de batterie
            update_battery_leds(voltage1);
            
            // Tension réelle (x2 pour compenser pont diviseur)
            float voltage_real = voltage1 * 2.0f;
            
            // Position actuelle
            Vector2DAndRotation pos = odometry.get_position();
            float pos_z_deg = pos.teta * 180.0f / M_PI;
            
            // Affichage unifié
            serial.printf("[%s] ENC:%ld,%ld SPD:%ld,%ld V:%.2f X:%.1f Y:%.1f Z:%.1f\r\n",
                         test_mode ? "MANUEL" : "AUTO",
                         enc1, enc2, speed1_ticks, speed2_ticks,
                         voltage_real, pos.x_y.x, pos.x_y.y, pos_z_deg);
            
            lastSend = HAL_GetTick();
        }

        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Callback des interruptions timer */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Timer générique [1Hz]
    if(htim->Instance == htim15.Instance){
        // Peut servir pour des tâches périodiques lentes
    }

    /* Asservissement [100Hz] */
    if(htim->Instance == htim17.Instance){
        if (!test_mode) {
            // Mise à jour de l'asservissement en mode asservi
            asserv.update_asserv();
        }
    }

    /* Odométrie [500Hz] */
    if(htim->Instance == htim6.Instance){
        // Mise à jour de l'odométrie dans tous les cas (pour lecture encodeurs)
        odometry.update_odometry();
    }
}

// Callback de réception UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rxByte == '\n' || rxByte == '\r') {
            if (cmd_idx > 0) {
                cmd_buffer[cmd_idx] = '\0';
                
                // Parser on (LED)
                if (strncmp(cmd_buffer, "on", 2) == 0) {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
                    serial.send(">> LED ON\r\n");
                }
                // Parser off (LED)
                else if (strncmp(cmd_buffer, "off", 3) == 0) {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
                    serial.send(">> LED OFF\r\n");
                }
                // Parser stop (arrêt d'urgence)
                else if (strncmp(cmd_buffer, "stop", 4) == 0) {
                    wheels[0].set_pwm(0);
                    wheels[1].set_pwm(0);
                    asserv.stop_asserv();
                    serial.send(">> MOTORS STOPPED\r\n");
                }
                // Parser mode manuel
                else if (strncmp(cmd_buffer, "mode manuel", 11) == 0) {
                    test_mode = true;
                    serial.send(">> MANUEL\r\n");
                    asserv.stop_asserv();
                    wheels[0].set_pwm(0);
                    wheels[1].set_pwm(0);
                }
                // Parser mode auto
                else if (strncmp(cmd_buffer, "mode auto", 9) == 0) {
                    test_mode = false;
                    serial.send(">> AUTO\r\n");
                    Vector2DAndRotation current_pos = odometry.get_position();
                    target_x = current_pos.x_y.x;
                    target_y = current_pos.x_y.y;
                    target_z = current_pos.teta * 180.0f / M_PI;
                    asserv.set_target_position(Vector2DAndRotation(target_x, target_y, current_pos.teta));
                    asserv.start_asserv();
                }
                // Parser M1:xxx (PWM -1000 à +1000) - uniquement en mode manuel
                else if (cmd_buffer[0] == 'M' && cmd_buffer[1] == '1' && cmd_buffer[2] == ':') {
                    if (test_mode) {
                        int val = 0;
                        sscanf(&cmd_buffer[3], "%d", &val);
                        if (val >= -1000 && val <= 1000) {
                            wheels[0].set_pwm(val);
                            serial.printf(">> M1=%d\r\n", val);
                        }
                    } else {
                        serial.send(">> Err: mode AUTO\r\n");
                    }
                }
                // Parser M2:xxx (PWM -1000 à +1000) - uniquement en mode manuel
                else if (cmd_buffer[0] == 'M' && cmd_buffer[1] == '2' && cmd_buffer[2] == ':') {
                    if (test_mode) {
                        int val = 0;
                        sscanf(&cmd_buffer[3], "%d", &val);
                        if (val >= -1000 && val <= 1000) {
                            wheels[1].set_pwm(val);
                            serial.printf(">> M2=%d\r\n", val);
                        }
                    } else {
                        serial.send(">> Err: mode AUTO\r\n");
                    }
                }
                // Parser X:xxx (Position X en mm) - uniquement en mode auto
                else if (cmd_buffer[0] == 'X' && cmd_buffer[1] == ':') {
                    if (!test_mode) {
                        float val = 0.0f;
                        sscanf(&cmd_buffer[2], "%f", &val);
                        target_x = val;
                        asserv.set_target_position(Vector2DAndRotation(target_x, target_y, target_z * M_PI / 180.0f));
                        serial.printf(">> X=%.1f\r\n", target_x);
                    } else {
                        serial.send(">> Err: mode MANUEL\r\n");
                    }
                }
                // Parser Y:xxx (Position Y en mm) - uniquement en mode auto
                else if (cmd_buffer[0] == 'Y' && cmd_buffer[1] == ':') {
                    if (!test_mode) {
                        float val = 0.0f;
                        sscanf(&cmd_buffer[2], "%f", &val);
                        target_y = val;
                        asserv.set_target_position(Vector2DAndRotation(target_x, target_y, target_z * M_PI / 180.0f));
                        serial.printf(">> Y=%.1f\r\n", target_y);
                    } else {
                        serial.send(">> Err: mode MANUEL\r\n");
                    }
                }
                // Parser Z:xxx (Rotation en degrés) - uniquement en mode auto
                else if (cmd_buffer[0] == 'Z' && cmd_buffer[1] == ':') {
                    if (!test_mode) {
                        float val = 0.0f;
                        sscanf(&cmd_buffer[2], "%f", &val);
                        target_z = val;
                        asserv.set_target_position(Vector2DAndRotation(target_x, target_y, target_z * M_PI / 180.0f));
                        serial.printf(">> Z=%.1f\r\n", target_z);
                    } else {
                        serial.send(">> Err: mode MANUEL\r\n");
                    }
                }
                
                cmd_idx = 0;  // Reset buffer
            }
        }
        else {
            if (cmd_idx < 19) {
                cmd_buffer[cmd_idx++] = rxByte;
            }
        }

        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
