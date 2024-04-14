/**
 * @file main.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This project incorporates basic function modules, including LED, USART, SDMMC, FATFS, CMSIS DSP. 
 * ! LED - LED can be used to indicate the running status of the program.
 * ! USART - USART can be used to print the running status of the program.
 * ! SDMMC - SDMMC can be used to read and write data to the SD card.
 * ! FATFS - FATFS can be used to manage the files on memory and SD card.
 * ! CMSIS DSP - CMSIS DSP can be used to perform digital signal processing operations.
 * @version 0.1
 * @date 2024-04-12
 */


/* DEPENDENCIES ----------------------------------------------- START*/

// BASIC MODULES
#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "led.h"
#include "usart.h"
#include "sdmmc_sd.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

// DSP MODULES
#include "arm_math.h"

// GPR-SPC 
#include "GPR-SPC.h"

/* DEPENDENCIES ----------------------------------------------- START*/


/* STATEMENTS ------------------------------------------------- START*/

/****************************************************************************************************/
/**
 *   System Clock Configuration
 *
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 550000000 (CPU main frequency 550MHz) 
 *            HCLK(Hz)                       = 275000000 (AXI and AHBs Clock)
 *            AHB Prescaler                  = 1 (AHB  Clock  275 MHz)
 *            D1 APB3 Prescaler              = 2 (APB3 Clock  137.5MHz)
 *            D2 APB1 Prescaler              = 2 (APB1 Clock  137.5MHz)
 *            D2 APB2 Prescaler              = 2 (APB2 Clock  137.5MHz)
 *            D3 APB4 Prescaler              = 2 (APB4 Clock  137.5MHz)
 *            HSE Frequency(Hz)              = 25000000  (external crystal on board)
 *            PLL_M                          = 10
 *            PLL_N                          = 220
 *            PLL_P                          = 1
 *
 *				CPU main frequency = HSE Frequency / PLL_M * PLL_N / PLL_P = 25M /10*220/1 = 550M
 */
/****************************************************************************************************/

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
	{
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 220;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
	// set SDMMC1 clock
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
	PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

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
/* HELPER FUNCTIONS --------------------------------------------- END*/


/* MAIN FUNCTION ---------------------------------------------- START*/

/**
 * @name main
 * @brief main function
 * @return int 
 * 
 */
int main(void)
{
	// initialize the system
	SCB_EnableICache();	  // enable ICache
	SCB_EnableDCache();	  // enable DCache
	HAL_Init();			  // init HAL library
	SystemClock_Config(); // configure system clock, main frequency 550MHz
	LED_Init();			  // init LED pin
	USART1_Init();		  // init USART1

	// GPR-SPC 
	GPRSPC(); // GPR-SPC test

	while (1)
	{
		LED1_Toggle;
		HAL_Delay(500);
	}
}
/* MAIN FUNCTION ------------------------------------------------ END*/


