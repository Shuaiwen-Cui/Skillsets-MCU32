/**
 * @file main.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This file is for cmsis-dsp library test
 * @version 0.1
 * @date 2024-03-18
 * 
 */ 
/*----------------------------------[DEPENDENCIES]---------------------------------*/
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
#include "nand.h"    
#include "ftl.h"  
#include "ff.h"
#include "exfuns.h"
/*----------------------------------<DEPENDENCIES>---------------------------------*/

/*----------------------------------[STATEMENTS]-----------------------------------*/
#define FFT_LENGTH		1024 		//FFT长度，默认是1024点FFT

float fft_inputbuf[FFT_LENGTH*2];	//FFT输入数组
float fft_outputbuf[FFT_LENGTH];	//FFT输出数组

u8 timeout;

/*----------------------------------<STATEMENTS>-----------------------------------*/


int main(void)
{
 	u8 key,t=0;
	float time; 
	u8 buf[50]; 
	u16 i; 
    arm_cfft_radix4_instance_f32 scfft;
 	u32 total,free;
	u8 res=0;
	
	Cache_Enable();                			//打开L1-Cache
	MPU_Memory_Protection();        		//保护相关存储区域
	HAL_Init();				        		//初始化HAL库
	Stm32_Clock_Init(160,5,2,4);  		    //设置时钟,400Mhz 
	delay_init(400);						//延时初始化
	uart_init(115200);						//串口初始化
	usmart_dev.init(200); 		    		//初始化USMART	
	LED_Init();								//初始化LED
	KEY_Init();								//初始化按键
	SDRAM_Init();                   		//初始化SDRAM
	LCD_Init();								//初始化LCD
    TIM3_Init(65535,200-1);        //1Mhz计数频率,最大计时6.5秒超出
    W25QXX_Init();				   		 	//初始化W25Q256
 	my_mem_init(SRAMIN);		    		//初始化内部内存池
	my_mem_init(SRAMEX);		    		//初始化外部内存池
	my_mem_init(SRAMDTCM);		    		//初始化CCM内存池 

   	POINT_COLOR=RED;
	LCD_ShowString(30,50,200,16,16,"Apollo STM32H7"); 
	LCD_ShowString(30,70,200,16,16,"FATFS TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2024/3/18");	 	 
	LCD_ShowString(30,130,200,16,16,"Use USMART for test");	
 	while(SD_Init())//检测不到SD卡
	{
		LCD_ShowString(30,150,200,16,16,"SD Card Error!");
		delay_ms(500);					
		LCD_ShowString(30,150,200,16,16,"Please Check! ");
		delay_ms(500);
		LED0_Toggle;//DS0闪烁
	}
    FTL_Init();
 	exfuns_init();							//为fatfs相关变量申请内存				 
  	f_mount(fs[0],"0:",1); 					//挂载SD卡 
 	res=f_mount(fs[1],"1:",1); 				//挂载FLASH.	
	if(res==0X0D)//FLASH磁盘,FAT文件系统错误,重新格式化FLASH
	{
		LCD_ShowString(30,150,200,16,16,"Flash Disk Formatting...");	//格式化FLASH
		res=f_mkfs("1:",FM_ANY,0,fatbuf,FF_MAX_SS);//格式化FLASH,1,盘符;1,不需要引导区,8个扇区为1个簇
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:ALIENTEK");	//设置Flash磁盘的名字为：ALIENTEK
			LCD_ShowString(30,150,200,16,16,"Flash Disk Format Finish");	//格式化完成
		}else LCD_ShowString(30,150,200,16,16,"Flash Disk Format Error ");	//格式化失败
		delay_ms(1000);
	}	
	res=f_mount(fs[2],"2:",1); 				//挂载NAND FLASH.	
	if(res==0X0D)//NAND FLASH磁盘,FAT文件系统错误,重新格式化NAND FLASH
	{
		LCD_ShowString(30,150,200,16,16,"NAND Disk Formatting...");//格式化NAND
		res=f_mkfs("2:",FM_ANY,0,fatbuf,FF_MAX_SS);	//格式化FLASH,2,盘符;1,不需要引导区,8个扇区为1个簇
		if(res==0)
		{
			f_setlabel((const TCHAR *)"2:NANDDISK");	//设置Flash磁盘的名字为：NANDDISK
			LCD_ShowString(30,150,200,16,16,"NAND Disk Format Finish");		//格式化完成
		}else LCD_ShowString(30,150,200,16,16,"NAND Disk Format Error ");	//格式化失败
		delay_ms(1000);
	}    
	LCD_Fill(30,150,240,150+16,WHITE);		//清除显示	
	while(exf_getfree("0:",&total,&free))	//得到SD卡的总容量和剩余容量
	{
		LCD_ShowString(30,150,200,16,16,"SD Card Fatfs Error!");
		delay_ms(200);
		LCD_Fill(30,150,240,150+16,WHITE);	//清除显示			  
		delay_ms(200);
		LED0_Toggle;//DS0闪烁
	}													  			    
 	POINT_COLOR=BLUE;//设置字体为蓝色	   
	LCD_ShowString(30,150,200,16,16,"FATFS OK!");	 
	LCD_ShowString(30,170,200,16,16,"SD Total Size:     MB");	 
	LCD_ShowString(30,190,200,16,16,"SD  Free Size:     MB"); 	    
 	LCD_ShowNum(30+8*14,170,total>>10,5,16);	//显示SD卡总容量 MB
 	LCD_ShowNum(30+8*14,190,free>>10,5,16);     //显示SD卡剩余容量 MB

	POINT_COLOR=RED; 
	LCD_ShowString(30,250,200,16,16,"Apollo STM32H7");	
	LCD_ShowString(30,270,200,16,16,"DISPLACEMENT ESTIMATION TEST");	
	LCD_ShowString(30,290,200,16,16,"SHUAIWEN CUI");
	LCD_ShowString(30,310,200,16,16,"2024/3/18");	
	LCD_ShowString(30,330,200,16,16,"KEY0:Run FFT");//显示提示信息 
	LCD_ShowString(30,360,200,16,16,"FFT runtime:");//显示FFT执行时间
 	POINT_COLOR=BLUE;		//设置字体为蓝色   
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//初始化scfft结构体，设定FFT相关参数

	printf("\r\nSD Card Fatfs OK!\r\n");

    while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			for(i=0;i<FFT_LENGTH;i++)//生成信号序列
			{
				 fft_inputbuf[2*i]=100+
				                   10*arm_sin_f32(2*PI*i/FFT_LENGTH)+
								   30*arm_sin_f32(2*PI*i*4/FFT_LENGTH)+
				                   50*arm_cos_f32(2*PI*i*8/FFT_LENGTH);	//生成输入信号实部
				 fft_inputbuf[2*i+1]=0;//虚部全部为0
			}
			__HAL_TIM_SET_COUNTER(&TIM3_Handler,0);//重设TIM3定时器的计数器值
			timeout=0;
			arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT计算（基4）
            time=__HAL_TIM_GET_COUNTER(&TIM3_Handler)+(u32)timeout*65536;//计算所用时间
			sprintf((char*)buf,"%0.3fms\r\n",time/1000);	
			LCD_ShowString(30+12*8,360,100,16,16,buf);	//显示运行时间		
			arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//把运算结果复数求模得幅值 
			printf("\r\n%d point FFT runtime:%0.3fms\r\n",FFT_LENGTH,time/1000);
			printf("FFT Result:\r\n");
			for(i=0;i<FFT_LENGTH;i++)
			{
				printf("fft_outputbuf[%d]:%f\r\n",i,fft_outputbuf[i]);
			}
		}else delay_ms(10);
		t++;
		printf("time:%d\r\n",t);
		if((t%10)==0)LED0_Toggle;		  
	}		 
}

