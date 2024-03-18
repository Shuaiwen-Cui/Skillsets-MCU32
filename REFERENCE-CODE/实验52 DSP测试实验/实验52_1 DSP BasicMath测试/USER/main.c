#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h"
#include "key.h"
#include "ltdc.h"
#include "lcd.h"
#include "sdram.h"
#include "usmart.h"
#include "pcf8574.h"
#include "mpu.h"
#include "malloc.h"
#include "w25qxx.h"
#include "sdmmc_sdcard.h"
#include "timer.h"
#include "arm_math.h"
/************************************************
 ALIENTEK ������STM32H7������ ʵ��52_1
 DSP BasicMath ʵ��-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

#define	DELTA	0.00005f		//���ֵ
 
//sin cos����
//angle:��ʼ�Ƕ�
//times:�������
//mode:0,��ʹ��DSP��;1,ʹ��DSP��
//����ֵ��0,�ɹ�;0XFF,����
u8 sin_cos_test(float angle,u32 times,u8 mode)
{
	float sinx,cosx;
	float result;
	u32 i=0;
	if(mode==0)
	{
		for(i=0;i<times;i++)
		{
			cosx=cosf(angle);			//��ʹ��DSP�Ż���sin��cos����
			sinx=sinf(angle);
			result=sinx*sinx+cosx*cosx; //������Ӧ�õ���1  
			result=fabsf(result-1.0f);	//�Ա���1�Ĳ�ֵ
			if(result>DELTA)return 0XFF;//�ж�ʧ�� 
			angle+=0.001f;				//�Ƕ�����
		}
	}else
	{
		for(i=0;i<times;i++)
		{
            
			cosx=arm_cos_f32(angle);	//ʹ��DSP�Ż���sin��cos����
			sinx=arm_sin_f32(angle);
			result=sinx*sinx+cosx*cosx; //������Ӧ�õ���1  
			result=fabsf(result-1.0f);	//�Ա���1�Ĳ�ֵ
			if(result>DELTA)return 0XFF;//�ж�ʧ�� 
			angle+=0.001f;				//�Ƕ�����
		}
	}
	return 0;//�������
} 
u8 timeout;

int main(void)
{
	float time; 
	u8 buf[50];
	u8 res;
	
	Cache_Enable();                			//��L1-Cache
	MPU_Memory_Protection();        		//������ش洢����
	HAL_Init();				        		//��ʼ��HAL��
	Stm32_Clock_Init(160,5,2,4);  		    //����ʱ��,400Mhz 
	delay_init(400);						//��ʱ��ʼ��
	uart_init(115200);						//���ڳ�ʼ��
	usmart_dev.init(200); 		    		//��ʼ��USMART	
	LED_Init();								//��ʼ��LED
	KEY_Init();								//��ʼ������
	SDRAM_Init();                   		//��ʼ��SDRAM
	LCD_Init();								//��ʼ��LCD
    TIM3_Init(65535,20000-1);               //10Khz����Ƶ��,����ʱ6.5�볬��
    POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Apollo STM32H7");	
	LCD_ShowString(30,70,200,16,16,"DSP BasicMath TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2018/6/14");	
	LCD_ShowString(30,150,200,16,16," No DSP runtime:");	//��ʾ��ʾ��Ϣ
	LCD_ShowString(30,190,200,16,16,"Use DSP runtime:");	//��ʾ��ʾ��Ϣ	
 	POINT_COLOR=BLUE;	//��������Ϊ��ɫ
	while(1)
	{
 		//��ʹ��DSP�Ż�
        __HAL_TIM_SET_COUNTER(&TIM3_Handler,0);//����TIM3��ʱ���ļ�����ֵ
		timeout=0;
		res=sin_cos_test(PI/6,200000,0);
        time=__HAL_TIM_GET_COUNTER(&TIM3_Handler)+(u32)timeout*65536;
		sprintf((char*)buf,"%0.1fms\r\n",time/10);
		if(res==0)LCD_ShowString(30+16*8,150,100,16,16,buf);	//��ʾ����ʱ��		
		else LCD_ShowString(30+16*8,150,100,16,16,"error��");	//��ʾ��ǰ�������	
		//ʹ��DSP�Ż�		
		__HAL_TIM_SET_COUNTER(&TIM3_Handler,0);//����TIM3��ʱ���ļ�����ֵ
		timeout=0;
		res=sin_cos_test(PI/6,200000,1);
		time=__HAL_TIM_GET_COUNTER(&TIM3_Handler)+(u32)timeout*65536;
		sprintf((char*)buf,"%0.1fms\r\n",time/10);
		if(res==0)LCD_ShowString(30+16*8,190,100,16,16,buf);	//��ʾ����ʱ��	
		else LCD_ShowString(30+16*8,190,100,16,16,"error��");	//��ʾ����	
		LED0_Toggle; 
	}		 
}

