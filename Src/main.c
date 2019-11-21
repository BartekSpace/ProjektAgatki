/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HTS221_AC_ADDRESS (0x5F<<1) // adres czujnika temp i wilg
#define CTRL_REG1 (0x20)
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define AV_CONF 0x10
#define STATUS_REG 0x27
#define T0_degC_x8 0x32
#define T1_degC_x8 0x33
#define T1_T0_msb 0x35
#define T0_OUT_L 0x3C
#define T0_OUT_H 0x3D
#define T1_OUT_L 0x3E
#define T1_OUT_H 0x3F

#define WHO_AM_I 0x0F

#define HUMIDITY_OUT_H 0x29
#define TEMP_OUT_H 0x2B
#define TEMP_OUT_L 0x2A
#define power_up 0x85
#define boot 0x0
#define interrupt_flag 0x44
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Value=0;
uint8_t Settings=1;
uint8_t Data=0;
int16_t val=0;
int16_t tmp = 0;
unsigned int res;
int test =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int read_T_OUT ( int8_t T_OUT_L, int8_t T_OUT_H )
{
	int16_t T_OUT;
	uint8_t tmp;
	int16_t  x;
	HAL_I2C_Mem_Read(&hi2c2,HTS221_AC_ADDRESS,T_OUT_H,1,&tmp,1,100);
	 T_OUT = tmp << 8;
	 HAL_I2C_Mem_Read(&hi2c2,HTS221_AC_ADDRESS,T_OUT_L,1,&tmp,1,100);

	 x=T_OUT;
	 T_OUT= x | tmp;

	 return T_OUT;

}

double calibration (int T_OUT)
{
	double T_DegC;
	uint8_t T1_T0msb;
	 HAL_I2C_Mem_Read(&hi2c2,HTS221_AC_ADDRESS,T1_T0_msb,1,&T1_T0msb,1,100);

uint8_t T0_degC_8;
uint8_t T1_degC_8;

HAL_I2C_Mem_Read(&hi2c2,HTS221_AC_ADDRESS,T0_degC_x8,1,&T0_degC_8,1,100);
HAL_I2C_Mem_Read(&hi2c2,HTS221_AC_ADDRESS,T1_degC_x8,1,&T1_degC_8,1,100);

int T0_OUT = read_T_OUT (T0_OUT_L, T0_OUT_H);
int T1_OUT = read_T_OUT (T1_OUT_L, T1_OUT_H);

double T0_degC = (T0_degC_8 + (1 << 8) * (T1_T0msb & 0x03)) / 8.0;
double T1_degC = (T1_degC_8 + (1 << 6) * (T1_T0msb & 0x0C)) / 8.0; // Value is in 3rd and fourth bit, so we only need to shift this value 6 more bits.


	T_DegC = (T0_degC + (T_OUT - T0_OUT) * (T1_degC - T0_degC) / (T1_OUT - T0_OUT));
return T_DegC;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ( GPIO_Pin == Interrupt_flag_Pin)
	{
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  			HAL_Delay(1000);
	}
}
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  Value=power_up;
  Settings=boot;
  uint8_t flag = interrupt_flag;
  HAL_I2C_Mem_Write(&hi2c2,HTS221_AC_ADDRESS,CTRL_REG1,1,&Value,1,100);

  HAL_I2C_Mem_Write(&hi2c2,HTS221_AC_ADDRESS,CTRL_REG2,1,&Settings,1,100);
  HAL_I2C_Mem_Write(&hi2c2,HTS221_AC_ADDRESS,CTRL_REG3,1,&flag,1,100);
//HAL_GPIO_WritePin(Interrupt_flag_GPIO_Port,Interrupt_flag_Pin,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  HAL_I2C_Mem_Read(&hi2c2,HTS221_AC_ADDRESS,TEMP_OUT_H,1,&Data,1,100);

	  				   tmp=Data<<8;
	  				   HAL_I2C_Mem_Read(&hi2c2,HTS221_AC_ADDRESS,TEMP_OUT_L,1,&Data,1,100);
	  				   val = tmp | Data ;


	  				  res = calibration(val);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
