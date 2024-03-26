#include "main.h"

UART_HandleTypeDef huart1;//UART_HandleTypeDef 结构体变量 - UART_HandleTypeDef structure variable

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	if(huart->Instance == USART1)
	{
		__HAL_RCC_USART1_CLK_ENABLE();//开启 USART1 时钟 - Enable USART1 clock

		GPIO_USART1_TX_CLK_ENABLE;//开启 USART1 TX 引脚的 GPIO 时钟 - Enable GPIO clock of USART1 TX pin
		GPIO_USART1_RX_CLK_ENABLE;//开启 USART1 RX 引脚的 GPIO 时钟 - Enable GPIO clock of USART1 RX pin

		GPIO_InitStruct.Pin = USART1_TX_PIN;//TX引脚 - TX pin
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;//复用推挽输出 - Alternate push-pull output
		GPIO_InitStruct.Pull = GPIO_PULLUP;//上拉 - Pull-up
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;//速度等级  - Speed level
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;//复用为USART1 - Multiplexed to USART1
		HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = USART1_RX_PIN;//RX引脚 - RX pin
		HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);		
	}
}

void lw_usart1_init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {

  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {

  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {

  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {

  }
}


/*************************************************************************************************
*	在有些场合，例如LVGL因为需要用__aeabi_assert或者TouchGFX，不能勾选 microLib 以使用printf - In some cases, such as LVGL, because __aeabi_assert or TouchGFX is needed, microLib cannot be selected to use printf
*	添加以下代码，让标准C库支持重定向fput - Add the following code to make the standard C library support redirecting fput
*  根据编译器，选择对应的代码即可 - According to the compiler, select the corresponding code
*************************************************************************************************/
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);	// 发送单字节数据 - Send single byte data
	return (ch);
}

