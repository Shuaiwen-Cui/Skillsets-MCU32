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
#include "disp_est.h"
/*----------------------------------<DEPENDENCIES>---------------------------------*/

/*----------------------------------[STATEMENTS]-----------------------------------*/
#define FFT_LENGTH		1024 		//FFT LENGTH BY DEFAULT 1024

float fft_inputbuf[FFT_LENGTH*2];	//FFT INPUT 
float fft_outputbuf[FFT_LENGTH];	//FFT OUTPUT

u8 timeout;

FRESULT ret; // Result code
FIL *fobj;	 // File object

/*----------------------------------<STATEMENTS>-----------------------------------*/

int main(void)
{
/*----------------------------------[STATEMENTS]-----------------------------------*/
 	u8 key,t=0;
	float time; 
	u8 buf[50]; 
	u16 i; 
    arm_cfft_radix4_instance_f32 scfft;
 	u32 total,free;
	u8 res=0;

	int x_base = 30;
	int x_offset = 0;
	int x_span = 240;
	int y_base = 50;
	int y_offset = 20;
	int y_span = 150;
	int show_width = 200;
	int show_height = 16;
	int font_size = 16;
	int num_digits = 5;

/*----------------------------------<[>STATEMENTS>-----------------------------------*/

/*----------------------------------[DEVICE INIT]-----------------------------------*/
	Cache_Enable();                			//Enable L1 Cache
	MPU_Memory_Protection();        		//Memory Protection Unit
	HAL_Init();				        		//Enable HAL
	Stm32_Clock_Init(160,5,2,4);  		    //Setup System Clock 400Mhz
	delay_init(400);						//Delay Init
	uart_init(115200);						//UART Init
	usmart_dev.init(200); 		    		//USMART Init
	LED_Init();								//LED Init
	KEY_Init();								//KEY Init
	SDRAM_Init();                   		//SDRAM Init
	LCD_Init();								//LCD Init
    TIM3_Init(65535,200-1);        			//1MHz counter frequency, maximum timing 6.5 seconds
    W25QXX_Init();				   		 	//W25QXX Init
 	my_mem_init(SRAMIN);		    		//Initialization of internal memory pool
	my_mem_init(SRAMEX);		    		//Initialization of external memory pool 
	my_mem_init(SRAMDTCM);		    		//Initialization of CCM memory pool
	SD_Init();								//SD Card Init
    FTL_Init();					            //FTL Init	
 	exfuns_init();							//Apply for memory for fatfs related variables	
/*----------------------------------<DEVICE INIT>-----------------------------------*/

/*----------------------------------[APPLICATION SETUP]-----------------------------*/
   	POINT_COLOR=RED;
	LCD_Clear(WHITE);
	LCD_ShowString(x_base,y_base,show_width,show_height,font_size,"DISPLACEMENT ESTIMATION");

	// STORAGE SETUP: SD CARD & FLASH & NAND FLASH
	// STORAGE - SD CARD
	f_mount(fs[0],"0:",1); 					//mount SD Card
	// STORAGE - FLASH
 	res=f_mount(fs[1],"1:",1); 				//mount FLASH.
	if(res==0X0D)//FLASH - FATFS system eroor, reformat FLASH
	{
		LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"Flash Disk Formatting...");	//reformat FLASH
		res=f_mkfs("1:",FM_ANY,0,fatbuf,FF_MAX_SS);//reformating FLASH, 1, disk mark; 1, no need for boot sector, 8 sectors for 1 cluster
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:CSW-FLASH");	//set the name of the Flash disk to: CSW-FLASH
			LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"Flash Disk Format Finish");	//formatting completed
		}else LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"Flash Disk Format Error ");	//formatting failed
		delay_ms(1000);
	}
	// STORAGE - NAND FLASH
	res=f_mount(fs[2],"2:",1); 				//mount NAND FLASH.
	if(res==0X0D)//NAND FLASH, FAT file system error, reformat NAND FLASH
	{
		LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"NAND Disk Formatting...");//reformating NAND FLASH
		res=f_mkfs("2:",FM_ANY,0,fatbuf,FF_MAX_SS);	//reformating FLASH, 2, disk mark; 1, no need for boot sector, 8 sectors for 1 cluster
		if(res==0)
		{
			f_setlabel((const TCHAR *)"2:CSW-NANDDISK");	//set the name of the NAND disk to: CSW-NANDDISK
			LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"NAND Disk Format Finish");		//formating completed
		}else LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"NAND Disk Format Error ");	//formatting failed
		delay_ms(1000);
	}   

	while(exf_getfree("0:",&total,&free))	//get the sizes of SD card, total and free
	{
		LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"SD Card Fatfs Error!");
		delay_ms(200);
		LCD_Fill(x_base,y_base + y_offset,x_span,y_span + font_size,WHITE);	//clear the display area			  
		delay_ms(200);
		LED0_Toggle;//DS0 blicking
	}
 	POINT_COLOR=BLUE;//set the font color to blue 
	LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"FATFS OK!");
	y_offset += 20;	 
	LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"SD Total Size:     MB");
	y_offset += 20;	 
	LCD_ShowString(x_base,y_base + y_offset,show_width,show_height,font_size,"SD  Free Size:     MB");
	x_offset = 8*14;
	y_offset -= 20; 	    
 	LCD_ShowNum(x_base + x_offset,y_base + y_offset,total>>10,num_digits,font_size);	//show the total capacity of SD card MB
	y_offset += 20;
 	LCD_ShowNum(x_base + x_offset,y_base + y_offset,free>>10,num_digits,font_size);     //show the free capacity of SD card MB

	printf("\r\nSD Card Fatfs OK!\r\n");

 	POINT_COLOR=BLUE;		//set the font color to blue  
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);// init the FFT structure


/*----------------------------------<APPLICATION SETUP>-----------------------------*/

/*-----------------------------------[APPLICATION LOOP]-----------------------------*/		
	while(1)
	{
		key = KEY_Scan(0);
		if (key == KEY0_PRES)
		{
			delay_ms(1000);
			printf("DISP EST TEST BUSY.\r\n");
		}
		else
		{
			delay_ms(1000);
			printf("DISP EST TEST IDLE.\r\n");
			disp_est();
		}
		t++;
		// printf("t:%d\r\n",t);
		if((t%2)==0)LED0_Toggle;
	}

/*-----------------------------------<APPLICATION LOOP>-----------------------------*/

}

