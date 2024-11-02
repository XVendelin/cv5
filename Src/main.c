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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"


#define 	LSM6DS0_DEVICE_ADDRESS		0x6A
#define 	LSM6DS0_DEVICE_ADDRESS_1	0x6B
#define 	LSM6DS0_WHO_AM_I_VALUE		0x68
#define 	LSM6DS0_WHO_AM_I_ADDRES		0x0F
#define 	LSM6DS0_ADDRESS_CTRL1		0x10

#define 	LSM6DS0_ADDRESS_X		0x28
#define 	LSM6DS0_ADDRESS_Y		0x2A
#define 	LSM6DS0_ADDRESS_Z		0x2C


void SystemClock_Config(void);
uint8_t addres = LSM6DS0_DEVICE_ADDRESS;
float acc[3];




void read_array(uint8_t *data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, addres, 0);
}

void get_acc(float* x, float* y, float* z)
{
	uint8_t data[6];
	int16_t xx, yy, zz;

	read_array(data, LSM6DS0_ADDRESS_X, 6);

	xx = ((uint16_t)data[1])<<8|data[0];
	yy = ((uint16_t)data[2])<<8|data[2];
	zz = ((uint16_t)data[3])<<8|data[4];

	*x = (xx>>4)/1000.0f;
	*y = (yy>>4)/1000.0f;
	*z = (zz>>4)/1000.0f;
}

uint8_t lsm6ds0_init(void)
{
	uint8_t status = 1;
	LL_mDelay(100);

	if(i2c_master_read_byte(LSM6DS0_DEVICE_ADDRESS, LSM6DS0_WHO_AM_I_ADDRES) == LSM6DS0_WHO_AM_I_VALUE)
	{
		status = 1;
	}
	else
	{
		if(i2c_master_read_byte(LSM6DS0_DEVICE_ADDRESS_1, LSM6DS0_WHO_AM_I_ADDRES) == LSM6DS0_WHO_AM_I_VALUE)
		{
			status = 1;
		}
		else
		{
			status = 0;
		}
	}
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  lsm6ds0_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   while (1) {
   // Main loop
	   get_acc(&acc[0], &acc[1], &acc[2]);
	   char buffer[20]; // Buffer to hold the string representation of the acceleration
	   sprintf(buffer, "Acceleration Z: %0.4f\r\n", acc[0]); // Format the string
	   USART2_SendString(buffer);
	   LL_mDelay(2000);
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
