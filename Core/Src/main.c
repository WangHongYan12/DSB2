/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buzzer/buzzer.h"
#include "uart2_motor_frame_tx/uart2_motor_frame_tx.h"
#include "oled\font.h"
#include "oled\oled.h"
#include "motor_rx\motor_rx.h"
#include <stdio.h>
#include <stdlib.h>
#include "linetracker_uart5/linetracker_uart5.h"
#include "wind\wind.h"
#include "key/key.h"
#include "imu_uart4/imu_uart4.h"
#include "yaw_pid_control/yaw_pid_control.h"
#include "yaw_pid_control/yaw_ramp.h"
#include "line_follow_pid/line_follow_pid.h"
#include "servo_control/servo_control.h"
#include "process.h"
#include "vision_parser/vision_parser.h"
#include "move_while_rotating/move_while_rotating.h"
#include "vision_parser/vision_align.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
    process_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    char buffer[20];
    servo_set_angle(SERVO_1, 60);
    servo_set_angle(SERVO_2, 225);
    process_0();
    process_1();
    process_2();
    process_3();
    process_4();
    process_5();
    process_6();
  while (1)
  {
      OLED_NewFrame();
      OLED_PrintString(4, 0, "电赛智能车 鸭敏电组", &font12x12, OLED_COLOR_REVERSED);
      OLED_DrawLine(0,14,128,14,OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 16, "KEY1:", &afont12x6, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 28, "KEY2:", &afont12x6, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 40, "KEY3:", &afont12x6, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(0, 52, "KEY4:", &afont12x6, OLED_COLOR_NORMAL);

      OLED_PrintString(24, 16, "朝向环", &font12x12, OLED_COLOR_NORMAL);
      OLED_PrintString(24, 28, "清零角", &font12x12, OLED_COLOR_NORMAL);
      OLED_PrintString(24, 40, "", &font12x12, OLED_COLOR_NORMAL);
      OLED_PrintString(24, 52, "发车", &font12x12, OLED_COLOR_REVERSED);

      snprintf(buffer, sizeof(buffer), "%d",motor_sum_odometers_x);
      OLED_PrintASCIIString(64, 16, buffer, &afont12x6, OLED_COLOR_NORMAL);
      OLED_PrintString(64, 28, "角度", &font12x12, OLED_COLOR_NORMAL);
      snprintf(buffer, sizeof(buffer), "%.2f",(float)imu_yaw_cdeg/100.0f);
      OLED_PrintASCIIString(88, 28, buffer, &afont12x6, OLED_COLOR_NORMAL);

      OLED_PrintString(64, 40, "速度", &font12x12, OLED_COLOR_NORMAL);
      snprintf(buffer, sizeof(buffer), "%d",last_set_speed[1]);
      OLED_PrintASCIIString(88, 40, buffer, &afont12x6, OLED_COLOR_NORMAL);

      OLED_PrintString(64, 52, "线数", &font12x12, OLED_COLOR_NORMAL);
      snprintf(buffer, sizeof(buffer), "%d",lt_digital[0]+lt_digital[1]+lt_digital[2]+lt_digital[3]+lt_digital[4]+lt_digital[5]+lt_digital[6]+lt_digital[7]);
      OLED_PrintASCIIString(100, 52, buffer, &afont12x6, OLED_COLOR_NORMAL);
      OLED_ShowFrame();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
