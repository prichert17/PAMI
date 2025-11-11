#include "control/motor.hpp"
#include "control/odometry.hpp"
#include "control/asservissement.hpp"
#include "main.hpp"
#include "utils/printf.hpp"
#include "ST_files/tim.h"
#include "ST_files/usart.h"
#include "ST_files/gpio.h"


void SystemClock_Config(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

SerialOut serial;

std::array<Wheel, 3> wheels = {
	Wheel(1, CONSTANTS::ENCODER_STEP_REV, CONSTANTS::WHEEL_RADIUS, 270.0),
	Wheel(2, CONSTANTS::ENCODER_STEP_REV, CONSTANTS::WHEEL_RADIUS, 30.0),
	Wheel(3, CONSTANTS::ENCODER_STEP_REV, CONSTANTS::WHEEL_RADIUS, 150.0)
};

Odometry odometry(wheels);

Asserv_Position asserv(&odometry, &wheels);

int main(void)
{
	HAL_Init();


    SystemClock_Config();


    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM8_Init();
    MX_TIM15_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_USART2_UART_Init();

    TIM1->CNT = (1<<15);
    TIM2->CNT = (1<<15);
    TIM3->CNT = (1<<15);

    odometry.attach_serial(&serial);
    asserv.attach_serial(&serial);

    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim15);
    HAL_TIM_Base_Start_IT(&htim16);

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

    asserv.set_target_position(Vector2DAndRotation(400, 0, 0));
    asserv.set_PID(250.0, 1000.0, 0.0);
    asserv.start_asserv();

    //asserv.set_PID(2500.0, 0.0, 0.0);
    //asserv.set_target_speed(Vector2DAndRotation(0, 0, 1.5));
    //asserv.start_asserv();

    while (1){

		//odometry.print_position();
		odometry.print_position();
		asserv.print_command();
		//serial.printf("ENC1 : %d\tENC2 : %d\tENC3 : %d\n",enc_1_val, enc_2_val, enc_3_val);
		//serial.printf_decimal((double)enc_1_val, 9);
		//serial.printf_decimal(cnt, 9);
		//HAL_Delay(200);

    }
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
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

	/** Initializes the CPU, AHB and APB buses clocks
	 */
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Update timer to avoid overflow [1Hz]
    if(htim->Instance == htim15.Instance){
    }

    /* Update speed [100Hz] */
    if(htim->Instance == htim16.Instance){
        asserv.update_asserv();
    }

    /* Odometry update [500Hz]*/
    if(htim->Instance == htim6.Instance){
		  odometry.update_odometry();
    }
}


void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
