#include "control/motor.hpp"
#include "control/odometry.hpp"
#include "control/asservissement.hpp"
#include "main.hpp"
#include "utils/printf.hpp"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include <cstring>
#include <cstdio>

void SystemClock_Config(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

SerialOut serial;

// Configuration pour 3 roues (mais on n'utilise que les 2 premières)
std::array<Wheel, 3> wheels = {
    Wheel(1, CONSTANTS::ENCODER_STEP_REV, CONSTANTS::WHEEL_RADIUS, 0.0),      // Roue gauche
    Wheel(2, CONSTANTS::ENCODER_STEP_REV, CONSTANTS::WHEEL_RADIUS, 180.0),    // Roue droite
    Wheel(3, CONSTANTS::ENCODER_STEP_REV, CONSTANTS::WHEEL_RADIUS, 90.0)      // Roue 3 (non utilisée)
};

Odometry odometry(wheels);
Asserv_Position asserv(&odometry, &wheels);

// Variables pour l'ADC
uint16_t adc1_value = 0;
uint16_t adc2_value = 0;

// Variables pour la réception UART
uint8_t rxByte;
uint8_t buffer[3] = {0}; // On garde les 3 derniers caractères

// Fonction pour communiquer avec l'autre microcontrôleur via UART1
void sendToOtherMCU(const char* message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100);
}

// Fonction pour recevoir des données de l'autre microcontrôleur via UART1
void receiveFromOtherMCU() {
    // La réception se fait par interruption via HAL_UART_RxCpltCallback
    // Cette fonction peut être utilisée pour traiter les données reçues si nécessaire
}

// Fonction pour lire les tensions ADC
void readADCValues() {
    // Lecture ADC1_IN15
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adc1_value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    
    // Lecture ADC2_IN13
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK) {
        adc2_value = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
}

// Fonction pour convertir la valeur ADC en tension (en mV)
float adcToVoltage(uint16_t adc_value) {
    // Assuming 12-bit ADC and 3.3V reference
    return (adc_value * 3300.0f) / 4095.0f;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_TIM15_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    // Initialisation des compteurs encodeurs au milieu de leur plage
    TIM2->CNT = (1<<15);
    TIM3->CNT = (1<<15);

    odometry.attach_serial(&serial);
    asserv.attach_serial(&serial);

    // Démarrage des timers
    HAL_TIM_Base_Start_IT(&htim6);   // Odométrie 500Hz
    HAL_TIM_Base_Start_IT(&htim15);  // Timer générique 1Hz
    HAL_TIM_Base_Start_IT(&htim17);  // Asservissement 100Hz

    // Démarrage des encodeurs
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    // Démarrage des PWM pour les 2 moteurs principaux
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);      // Moteur 1
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);   // Moteur 1 (canal complémentaire)
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);     // Moteur 2
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);  // Moteur 2 (canal complémentaire)
    
    // Moteur 3 en réserve (TIM8)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);

    // Configuration de l'asservissement
    asserv.set_target_position(Vector2DAndRotation(400, 0, 0));
    asserv.set_PID(250.0, 1000.0, 0.0);
    asserv.start_asserv();

    // Démarrage de la réception UART sur interruption (1 octet à la fois)
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);

    // Message de démarrage au autre MCU
    sendToOtherMCU("System initialized\r\n");

    while (1) {
        odometry.print_position();
        asserv.print_command();
        
        // Lecture périodique des ADC
        readADCValues();
        
        // Envoi des valeurs ADC à l'autre MCU toutes les secondes environ
        static uint32_t lastADCSend = 0;
        if (HAL_GetTick() - lastADCSend > 1000) {
            char buffer_adc[100];
            sprintf(buffer_adc, "ADC1: %.2fmV, ADC2: %.2fmV\r\n", 
                    adcToVoltage(adc1_value), adcToVoltage(adc2_value));
            sendToOtherMCU(buffer_adc);
            lastADCSend = HAL_GetTick();
        }

        HAL_Delay(200);
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Timer générique [1Hz]
    if(htim->Instance == htim15.Instance){
        // Actions périodiques toutes les secondes
    }

    // Asservissement [100Hz]
    if(htim->Instance == htim17.Instance){
        asserv.update_asserv();
    }

    // Odométrie [500Hz]
    if(htim->Instance == htim6.Instance){
        odometry.update_odometry();
    }
}

// Callback de réception UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Décalage du buffer pour stocker le nouveau caractère
        buffer[0] = buffer[1];
        buffer[1] = buffer[2];
        buffer[2] = rxByte;

        // Détection de "on" (fin de séquence 'o', 'n')
        // On regarde les deux derniers caractères reçus
        if (buffer[1] == 'o' && buffer[2] == 'n')
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            sendToOtherMCU("led allumée\r\n");
        }
        
        // Détection de "off" (fin de séquence 'o', 'f', 'f')
        // On regarde les trois derniers caractères reçus
        else if (buffer[0] == 'o' && buffer[1] == 'f' && buffer[2] == 'f')
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            sendToOtherMCU("led éteinte\r\n");
        }
        
        // Relance la réception pour le prochain caractère
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
