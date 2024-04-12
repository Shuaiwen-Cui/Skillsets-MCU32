/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*  @date    2023-4-7
	*	@author  反客科技
   ************************************************************************************************
   *  @description
	*
	*	实验平台：反客STM32H723ZGT6核心板 （型号：FK723M1-ZGT6）
	*	淘宝地址：https://shop212360197.taobao.com
	*	QQ交流群：536665479
	*
>>>>> 功能说明：
	*
	*	点亮LED，使用HAL库自带的延时函数实现闪烁
	*
	************************************************************************************************
***/

#include "main.h"
#include "led.h"



/********************************************** 函数声明 *******************************************/

void SystemClock_Config(void);		// 时钟初始化

/***************************************************************************************************
*	函 数 名: main
*	入口参数: 无
*	返 回 值: 无
*	函数功能: 运行主程序
*	说    明: 无
****************************************************************************************************/

int main(void)
{
 	SCB_EnableICache();		// 使能ICache
	SCB_EnableDCache();		// 使能DCache
	HAL_Init();					// 初始化HAL库
	SystemClock_Config();	// 配置系统时钟，主频550MHz
	LED_Init();					// 初始化LED引脚
	
	while (1)
	{
		LED1_Toggle;
		HAL_Delay(1000);	
	}
}


/****************************************************************************************************/
/**
  *   系统时钟配置：
  *
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 550000000 (CPU 主频 550MHz)
  *            HCLK(Hz)                       = 275000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 1 (AHB  Clock  275 MHz)
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  137.5MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  137.5MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  137.5MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  137.5MHz)
  *            HSE Frequency(Hz)              = 25000000  (外部晶振频率)
  *            PLL_M                          = 10
  *            PLL_N                          = 220
  *            PLL_P                          = 1
  *
  *				CPU主频 = HSE Frequency / PLL_M * PLL_N / PLL_P = 25M /10*220/1 = 550M
  *
  */   
/****************************************************************************************************/

void SystemClock_Config(void)	// 该函数由cubeMX生成，例程里所有外设时钟都放在这个函数
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
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

