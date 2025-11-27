#include "main.hpp"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "control/motor.hpp"
#include "utils/printf.hpp"
#include <cstring>
#include <array>

void SystemClock_Config(void);

// Instance de SerialOut pour la communication UART
SerialOut serial;

// Variables globales
uint8_t rxByte;
char cmd_buffer[20] = {0};
uint8_t cmd_idx = 0;

// Variables ADC
uint32_t adc_values[2];  // Buffer pour stocker les valeurs ADC

// Moteurs utilisant la classe Wheel
std::array<Wheel, 3> wheels = {
    Wheel(1, 1200, 29.0f, 0.0f),     // Moteur 1
    Wheel(2, 1200, 29.0f, 180.0f),   // Moteur 2
    Wheel(3, 1200, 29.0f, 0.0f)      // Moteur 3 (réserve)
};



int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialisation des périphériques
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM16_Init();
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
    TIM2->CNT = 32768;
    TIM3->CNT = 32768;

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

    // Message de démarrage
    serial.send("\r\n=== TEST SIMPLE MOTEURS ===\r\n");
    serial.send("Commandes:\r\n");
    serial.send("  M1:xxx  -> Moteur 1 PWM 0-255\r\n");
    serial.send("  M2:xxx  -> Moteur 2 PWM 0-255\r\n");
    serial.send("  stop    -> Arrêt d'urgence\r\n");
    serial.send("  on/off  -> LED\r\n");
    serial.send("===========================\r\n\r\n");

    while (1) {
        // Calcul de la vitesse toutes les 500ms
        static uint32_t lastSend = 0;
        
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
            adc_values[0] = 0;
            if (HAL_ADC_Start(&hadc1) == HAL_OK) {
                if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                    adc_values[0] = HAL_ADC_GetValue(&hadc1);
                }
                HAL_ADC_Stop(&hadc1);
            }
            
            // Lecture ADC2 (PA5 - Canal 13)
            adc_values[1] = 0;
            if (HAL_ADC_Start(&hadc2) == HAL_OK) {
                if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK) {
                    adc_values[1] = HAL_ADC_GetValue(&hadc2);
                }
                HAL_ADC_Stop(&hadc2);
            }
            
            // Conversion en tension (3.3V de référence, résolution 12 bits = 4095)
            float voltage1 = (adc_values[0] * 3.3f) / 4095.0f;
            float voltage2 = (adc_values[1] * 3.3f) / 4095.0f;
            
            // Affichage avec printf formaté
            serial.printf("ENC1:%6ld | ENC2:%6ld | SPD1:%5ld | SPD2:%5ld ticks/s | ADC1:%4lu (%.2fV) | ADC2:%4lu (%.2fV)\r\n",
                         enc1, enc2, speed1_ticks, speed2_ticks,
                         adc_values[0], voltage1, adc_values[1], voltage2);
            
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
                    serial.send(">> MOTORS STOPPED\r\n");
                }
                // Parser M1:xxx (PWM -1000 à +1000)
                else if (cmd_buffer[0] == 'M' && cmd_buffer[1] == '1' && cmd_buffer[2] == ':') {
                    int val = 0;
                    sscanf(&cmd_buffer[3], "%d", &val);
                    if (val >= -1000 && val <= 1000) {
                        wheels[0].set_pwm(val);
                        serial.printf(">> Moteur 1 = %d\r\n", val);
                    }
                }
                // Parser M2:xxx (PWM -1000 à +1000)
                else if (cmd_buffer[0] == 'M' && cmd_buffer[1] == '2' && cmd_buffer[2] == ':') {
                    int val = 0;
                    sscanf(&cmd_buffer[3], "%d", &val);
                    if (val >= -1000 && val <= 1000) {
                        wheels[1].set_pwm(val);
                        serial.printf(">> Moteur 2 = %d\r\n", val);
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
