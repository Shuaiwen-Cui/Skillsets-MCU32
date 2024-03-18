#include "sys.h"
#include "delay.h"
#include "usart.h" 
/************************************************
 ALIENTEK 阿波罗STM32H7开发板 实验0
 新建工程实验-HAL库版本
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

void Delay(__IO uint32_t nCount);
void Delay(__IO uint32_t nCount)
{
	while(nCount--){}
}

int main(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	Cache_Enable();                             //打开L1-Cache
	HAL_Init();				                    //初始化HAL库
	Stm32_Clock_Init(160,5,2,4);                //设置时钟,400Mhz 
	delay_init(400);
	uart_init(115200);
	__HAL_RCC_GPIOB_CLK_ENABLE();               //开启GPIOB时钟

	GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1;     //PB0,1
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

	while(1)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	//PB0置0
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);	//PB1置0
		Delay(0x1FFFFFF);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置1
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1
		Delay(0x1FFFFFF);

	}
}
