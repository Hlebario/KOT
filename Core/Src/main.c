/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
const double J2L = 57.0; // Length of J2 (57 - 2.35)mm
const double J3L = 110.0; // Length of J3 110mm

const double Y_Rest =  70.0;
const double Z_Rest = -80.0;

const double J3_LegAngle = 15.4;

const double lines[62][4] = {{0.0, 0.0, 40.0, 1000},

                            {-30.0, 40.0, 20.0, 200},
                            {-30.0, 40.0, -20.0, 200},
                            {60.0, 40.0, -20.0, 1600},
                            {60.0, 60.0, 20.0, 200},

                            {-30.0, 30.0, 20.0, 200},
                            {-30.0, 30.0, -20.0, 200},
                            {60.0, 30.0, -20.0, 1600},
                            {60.0, 50.0, 20.0, 200},

                            {-30.0, 20.0, 20.0, 200},
                            {-30.0, 20.0, -20.0, 200},
                            {60.0, 20.0, -20.0, 1600},
                            {60.0, 40.0, 20.0, 200},

                            {-30.0, 10.0, 20.0, 200},
                            {-30.0, 10.0, -20.0, 200},
                            {60.0, 10.0, -20.0, 1600},
                            {60.0, 30.0, 20.0, 200},

                            {-30.0, 0.0, 20.0, 200},
                            {-30.0, 0.0, -20.0, 200},
                            {60.0, 0.0, -20.0, 1600},
                            {60.0, 20.0, 20.0, 200},



                            {-30.0, 40.0, 20.0, 200},
                            {-30.0, 40.0, -20.0, 200},
                            {-30.0, 0.0, -20.0, 800},
                            {-30.0, 20.0, 20.0, 200},

                            {-20.0, 40.0, 20.0, 200},
                            {-20.0, 40.0, -20.0, 200},
                            {-20.0, 0.0, -20.0, 800},
                            {-20.0, 20.0, 20.0, 200},

                            {-10.0, 40.0, 20.0, 200},
                            {-10.0, 40.0, -20.0, 200},
                            {-10.0, 0.0, -20.0, 800},
                            {-10.0, 20.0, 20.0, 200},

                            {0.0, 40.0, 20.0, 200},
                            {0.0, 40.0, -20.0, 200},
                            {0.0, 0.0, -20.0, 800},
                            {0.0, 20.0, 20.0, 200},

                            {10.0, 40.0, 20.0, 200},
                            {10.0, 40.0, -20.0, 200},
                            {10.0, 0.0, -20.0, 800},
                            {10.0, 20.0, 20.0, 200},

                            {20.0, 40.0, 0.0, 200},
                            {20.0, 40.0, -20.0, 200},
                            {20.0, 0.0, -20.0, 800},
                            {20.0, 20.0, 0.0, 200},

                            {30.0, 40.0, 0.0, 200},
                            {30.0, 40.0, -20.0, 200},
                            {30.0, 0.0, -20.0, 800},
                            {30.0, 20.0, 0.0, 200},

                            {40.0, 40.0, 0.0, 200},
                            {40.0, 40.0, -20.0, 200},
                            {40.0, 0.0, -20.0, 800},
                            {40.0, 20.0, 0.0, 200},

                            {50.0, 40.0, 0.0, 200},
                            {50.0, 40.0, -20.0, 200},
                            {50.0, 0.0, -20.0, 800},
                            {50.0, 20.0, 0.0, 200},

                            {60.0, 40.0, 0.0, 200},
                            {60.0, 40.0, -20.0, 200},
                            {60.0, 0.0, -20.0, 800},
                            {60.0, 20.0, 0.0, 200},

                            {0.0, 0.0, 40.0, 200}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void CartesianMove(double X, double Y, double Z);
void UpdatePosition(double J1, double J2, double J3);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int i=0;
	for(i=0; i<len; i++) ITM_SendChar((*ptr++));
	return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //while (MPU6050_Init(&hi2c1) == 1);

  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_3);



 void UpdatePosition(double J1, double J2, double J3)
  {
    // MOVE TO POSITION
	TIM2 ->CCR1 = (90 - J1)*38.33333333333333+1600;
	TIM2 ->CCR2 = (90 - J2)*38.33333333333333+1600;
	TIM2 ->CCR3 = (J3 + J3_LegAngle - 90)*38.33333333333333+1600;

	TIM2 ->CCR4 = (90 - J1)*38.33333333333333+1600;
	TIM3 ->CCR1 = (90 - J2)*38.33333333333333+1600;
	TIM3 ->CCR2 = (J3 + J3_LegAngle - 90)*38.33333333333333+1600;

	TIM3 ->CCR3 = (90 - J1)*38.33333333333333+1600;
	TIM3 ->CCR4 = (90 - J2)*38.33333333333333+1600;
	TIM4 ->CCR1 = (J3 + J3_LegAngle - 90)*38.33333333333333+16000;

	TIM4 ->CCR2 = (90 - J1)*38.33333333333333+1600;
	TIM4 ->CCR3 = (90 - J2)*38.33333333333333+1600;
	TIM5 ->CCR3 = (J3 + J3_LegAngle - 90)*38.33333333333333+1600;
	HAL_Delay(200);
  }

  void CartesianMove(double X, double Y, double Z)
    {
    // OFFSET TO REST POSITION
    Y += Y_Rest;
    Z += Z_Rest;

    // CALCULATE INVERSE KINEMATIC SOLUTION
    double J1 = atan(X / Y) * (180 / M_PI);
    double H = sqrt((Y * Y) + (X * X));
    double L = sqrt((H * H) + (Z * Z));
    double J3 = acos(   ((J2L * J2L) + (J3L * J3L) - (L * L))   /   (2 * J2L * J3L)   ) * (180 / M_PI);
    double B = acos(   ((L * L) + (J2L * J2L) - (J3L * J3L))   /   (2 * L * J2L)   ) * (180 / M_PI);
    double A = atan(Z / H) * (180 / M_PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
    double J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'

    UpdatePosition(J1, J2, J3);
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  for(int commandStep = 0; commandStep < 62; commandStep++)
	  {
		  double xMove = lines[commandStep][0];
		  double yMove = lines[commandStep][1];
		  double zMove = lines[commandStep][2];

		  CartesianMove(xMove, yMove, zMove);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);*/
	 //MPU6050_Read_All(&hi2c1, &MPU6050);
	 //HAL_Delay (100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
