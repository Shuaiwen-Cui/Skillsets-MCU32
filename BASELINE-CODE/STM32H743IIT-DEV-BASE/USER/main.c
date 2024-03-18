#include "sys.h"
#include "delay.h"
#include "usart.h" 
/************************************************
 ALIENTEK ������STM32H7������ ʵ��0
 �½�����ʵ��-HAL��汾
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

void Delay(__IO uint32_t nCount);
void Delay(__IO uint32_t nCount)
{
	while(nCount--){}
}

int main(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	Cache_Enable();                             //��L1-Cache
	HAL_Init();				                    //��ʼ��HAL��
	Stm32_Clock_Init(160,5,2,4);                //����ʱ��,400Mhz 
	delay_init(400);
	uart_init(115200);
	__HAL_RCC_GPIOB_CLK_ENABLE();               //����GPIOBʱ��

	GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1;     //PB0,1
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //�������
	GPIO_Initure.Pull=GPIO_PULLUP;              //����
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

	while(1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0��0
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);	//PB1��0
		Delay(0x1FFFFFF);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB0��1
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB1��1
		Delay(0x1FFFFFF);

	}
}
