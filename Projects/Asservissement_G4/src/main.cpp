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
char cmd_buffer[50] = {0};
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
            int volt_decimale = (int)((voltage1*2) * 100); // *100 pour 2 chiffres

            // Position actuelle
            Vector2DAndRotation pos = odometry.get_position();
            float pos_z_deg = pos.teta * 180.0f / M_PI;
            
            int X_int = (int)pos.x_y.x;
            int Y_int = (int)pos.x_y.y;
            int Z_int = (int)pos_z_deg;
            
            // Affichage unifié
            serial.printf("[%s] ENC:%ld,%ld SPD:%ld,%ld V:%d X:%d Y:%d Z:%d\r\n",
                         test_mode ? "MANUEL" : "AUTO",
                         enc1, enc2, speed1_ticks, speed2_ticks,
                         volt_decimale, X_int, Y_int, Z_int);
            
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
        // Si on reçoit un caractère de fin de ligne (\n ou \r)
        if (rxByte == '\n' || rxByte == '\r') {
            if (cmd_idx > 0) {
                
                // 1. Fermeture propre de la chaine brute
                cmd_buffer[cmd_idx] = '\0';

                // 2. LE NETTOYEUR (Supprime les 0 et les caractères invisibles du début)
                char *p_cmd = cmd_buffer; // Par défaut, on pointe au début
                
                // On parcourt le buffer pour trouver la première vraie lettre (ASCII > 32)
                for(int i = 0; i < cmd_idx; i++) {
                    if (cmd_buffer[i] > 32) { // 32 = Espace. Donc on cherche une lettre ou un chiffre.
                        p_cmd = &cmd_buffer[i];
                        break; // Trouvé ! On arrête de chercher.
                    }
                }

                // (Optionnel) Debug pour voir ce qui est vraiment traité
                // serial.printf("CMD: [%s]\r\n", p_cmd);

                // -----------------------------------------------------------
                // 3. ANALYSE DES COMMANDES (On utilise p_cmd maintenant !)
                // -----------------------------------------------------------

                // Parser ON (LED)
                if (strncmp(p_cmd, "on", 2) == 0) {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
                    serial.send(">> LED ON\r\n");
                }
                // Parser OFF (LED)
                else if (strncmp(p_cmd, "off", 3) == 0) {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
                    serial.send(">> LED OFF\r\n");
                }
                // Parser STOP (Arrêt d'urgence)
                else if (strncmp(p_cmd, "stop", 4) == 0) {
                    wheels[0].set_pwm(0);
                    wheels[1].set_pwm(0);
                    asserv.stop_asserv();
                    serial.send(">> MOTORS STOPPED\r\n");
                }
                // Parser MODE MANUEL
                else if (strncmp(p_cmd, "mode manuel", 11) == 0) {
                    test_mode = true;
                    serial.send(">> MANUEL\r\n");
                    asserv.stop_asserv();
                    wheels[0].set_pwm(0);
                    wheels[1].set_pwm(0);
                }
                // Parser MODE AUTO
                else if (strncmp(p_cmd, "mode auto", 9) == 0) {
                    test_mode = false;
                    serial.send(">> AUTO\r\n");
                    // On verrouille la position cible sur la position actuelle pour ne pas partir en fou
                    Vector2DAndRotation current_pos = odometry.get_position();
                    target_x = current_pos.x_y.x;
                    target_y = current_pos.x_y.y;
                    target_z = current_pos.teta * 180.0f / M_PI;
                    asserv.set_target_position(Vector2DAndRotation(target_x, target_y, current_pos.teta));
                    asserv.start_asserv();
                }
                // Parser M1:xxx (PWM)
                else if (p_cmd[0] == 'M' && p_cmd[1] == '1' && p_cmd[2] == ':') {
                    if (test_mode) {
                        int val = 0;
                        sscanf(&p_cmd[3], "%d", &val); // Lecture depuis p_cmd
                        if (val >= -1000 && val <= 1000) {
                            wheels[0].set_pwm(val);
                            serial.printf(">> M1=%d\r\n", val);
                        }
                    } else {
                        serial.send(">> Err: mode AUTO\r\n");
                    }
                }
                // Parser M2:xxx (PWM)
                else if (p_cmd[0] == 'M' && p_cmd[1] == '2' && p_cmd[2] == ':') {
                    if (test_mode) {
                        int val = 0;
                        sscanf(&p_cmd[3], "%d", &val);
                        if (val >= -1000 && val <= 1000) {
                            wheels[1].set_pwm(val);
                            serial.printf(">> M2=%d\r\n", val);
                        }
                    } else {
                        serial.send(">> Err: mode AUTO\r\n");
                    }
                }
                // Parser X:xxx (Position)
                else if (p_cmd[0] == 'X' && p_cmd[1] == ':') {
                    if (!test_mode) {
                        float val = 0.0f;
                        sscanf(&p_cmd[2], "%f", &val);
                        target_x = val;
                        asserv.set_target_position(Vector2DAndRotation(target_x, target_y, target_z * M_PI / 180.0f));
                        serial.printf(">> X=%.1f\r\n", target_x);
                    } else {
                        serial.send(">> Err: mode MANUEL\r\n");
                    }
                }
                // Parser Y:xxx (Position)
                else if (p_cmd[0] == 'Y' && p_cmd[1] == ':') {
                    if (!test_mode) {
                        float val = 0.0f;
                        sscanf(&p_cmd[2], "%f", &val);
                        target_y = val;
                        asserv.set_target_position(Vector2DAndRotation(target_x, target_y, target_z * M_PI / 180.0f));
                        serial.printf(">> Y=%.1f\r\n", target_y);
                    } else {
                        serial.send(">> Err: mode MANUEL\r\n");
                    }
                }
                // Parser Z:xxx (Rotation)
                else if (p_cmd[0] == 'Z' && p_cmd[1] == ':') {
                    if (!test_mode) {
                        float val = 0.0f;
                        sscanf(&p_cmd[2], "%f", &val);
                        target_z = val;
                        asserv.set_target_position(Vector2DAndRotation(target_x, target_y, target_z * M_PI / 180.0f));
                        serial.printf(">> Z=%.1f\r\n", target_z);
                    } else {
                        serial.send(">> Err: mode MANUEL\r\n");
                    }
                }
                
                // 4. NETTOYAGE FINAL
                cmd_idx = 0;
                memset(cmd_buffer, 0, sizeof(cmd_buffer)); // Reset total du buffer
            }
        }
        else {
            // Remplissage du buffer
            if (cmd_idx < 49) { //buffer de 50
                cmd_buffer[cmd_idx++] = rxByte;
            } else {
                 cmd_idx = 0; // Sécurité anti-débordement
            }
        }

        // Relance l'écoute
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}
// Gère les erreurs UART (Overrun, Noise, Framing...)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // En cas d'erreur (souvent Overrun), on relance simplement l'écoute
        // Sinon l'UART reste bloqué à jamais
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
