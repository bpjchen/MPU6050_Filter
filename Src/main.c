/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "mpu6050.h"
#include "math.h"
#include "IIC.h"
#include "imu.h"
#include "SW_Wave.h"
#include "filter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t DATA_PRINTF = 0;
uint8_t IMU_CALC = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*******************************************硬件连接*********************************************
模块：采用的是淘宝上十多块钱的MPU6050模块（只支持IIC通信）
连接：
     PB6---SCL
     PB7---SDA
     5V---VCC
     GND---GND
     其他引脚悬空
注：通信采用的是模拟IIC，换成其它IO同样适用（本工程仅限于F4系列，若要用在其它
系列要在IIC.h文件中作相当的改动）
**********************************************END***********************************************/

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();  //MPU6050初始化
  HAL_TIM_Base_Start_IT(&htim3);  //使能TIM3中断，溢出时间为1ms
  IIR_Filter_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
      if(IMU_CALC)
      {
          IMU_CALC = 0;
          
          HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
          MPU6050_Read();
          MahonyAHRSupdateIMU(&IMU_Data);
          
      }
      if(DATA_PRINTF)
      {
          DATA_PRINTF = 0;
          
//          printf("\n\r acc offset: %d, %d, %d \n\r", MPU_Data.ax_offset, MPU_Data.ay_offset, MPU_Data.az_offset);
//          printf("\n\r gyro offset: %d, %d, %d \n\r", MPU_Data.gx_offset, MPU_Data.gy_offset, MPU_Data.gz_offset);
          
//          printf("\n\r acc: %d, %d, %d \n\r", MPU_Data.Acc.X, MPU_Data.Acc.Y, MPU_Data.Acc.Z);  //加速度原始数据
//          printf("\n\r gyro: %d, %d, %d \n\r", MPU_Data.gx, MPU_Data.gy, MPU_Data.gz);  //陀螺仪原始数据
//          printf("\n\r %d \n\r", MPU_Data.ay);
          
//          printf("\n\r acc: %f, %f, %f \n\r", IMU_Data.ax, IMU_Data.ay, IMU_Data.az);
//          printf("\n\r gyro: %f, %f, %f \n\r", IMU_Data.wx, IMU_Data.wy, IMU_Data.wz);
//          printf("\n\r temp: %d \n\r", OFFSET);
          
          printf("\n\r %f, %f, %f \n\r", IMU_Data.yaw, IMU_Data.pit, IMU_Data.rol);
          
//          SwDataWaveUpdate();

          
      }
      
      
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t ms1 = 0, ms50 = 0;
    if(htim->Instance == TIM3)
    {
        ms1++;
        ms50++;
        if(ms50 >= 50)
        {
            ms50 = 0;
            DATA_PRINTF = 1;
        }
        if(ms1 >= 1)
        {
            ms1 = 0;
            IMU_CALC = 1;
        }
        
    }
    
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
