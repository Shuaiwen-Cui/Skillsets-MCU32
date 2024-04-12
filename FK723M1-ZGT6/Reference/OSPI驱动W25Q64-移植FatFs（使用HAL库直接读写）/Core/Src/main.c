/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*  @date    2023-4-10
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
	*	1.点亮LED，使用HAL库自带的延时函数实现闪烁
	*   2.移植FatFs，进行简单的测试
	*	3.OSPI间接模式，使用库函数直接读写，
	*	4.默认配置QSPI驱动时钟为137.5M
	*
>>>>> 串口打印说明：
	*
	*	USART1使用的是PA9/PA10，串口波特率115200
	*	
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "usart.h"
#include "ff.h"

/********************************************** 变量定义 *******************************************/

FATFS 	sFLASH_FatFs; 					// 文件系统对象
FRESULT 	MyFile_Res;       			// 操作结果 
BYTE 		sFLASH_ReadBuffer[1024];	//要读出的数据


//	函数：FatFs_Check
//	功能：进行FatFs文件系统的挂载
//
void FatFs_Check(void)	//判断FatFs是否挂载成功，若没有创建FatFs则格式化SD卡
{
	BYTE work[FF_MAX_SS]; 
	
	MyFile_Res = f_mount(&sFLASH_FatFs,"1:",1);	//初始化SPI Flash，并挂载FatFs
	
	if (MyFile_Res == FR_OK)	//判断是否挂载成功
	{
		printf("\r\nSPI Flash文件系统挂载成功\r\n");
	}
	else		
	{
		printf("SPI Flash还未创建文件系统，即将格式化\r\n");
		
		MyFile_Res = f_mkfs("1:",FM_FAT,0,work,sizeof work);		//格式化，FAT32，簇默认大小16K
		
		if (MyFile_Res == FR_OK)		//判断是否格式化成功
			printf("SPI Flash格式化成功！\r\n");
		else
			printf("格式化失败，请检查或更换SPI Flash！\r\n");
	}
}

//	函数：FatFs_GetVolume
//	功能：计算设备的容量，包括总容量和剩余容量
//

void  FatFs_GetVolume(void)	// 计算设备容量
{
	FATFS *fs;		//定义结构体指针
	uint32_t sFLASH_CardCapacity = 0;		//SD卡的总容量
	uint32_t sFLASH_FreeCapacity = 0;		//SD卡空闲容量
	DWORD fre_clust, fre_sect, tot_sect; 	//空闲簇，空闲扇区数，总扇区数

	f_getfree("1:",&fre_clust,&fs);			//获取设备剩余的簇

	tot_sect = (fs->n_fatent-2) * fs->csize;	//总扇区数量 = 总的簇 * 每个簇包含的扇区数
	fre_sect = fre_clust * fs->csize;			//计算剩余的可用扇区数	   

	sFLASH_CardCapacity = tot_sect * 4096 / 1024;	//总容量 = 总扇区数 * 每扇区的字节数 / 1K
	sFLASH_FreeCapacity = fre_sect * 4096 / 1024;	//计算剩余的容量，单位为KB

	printf("-------------------获取设备容量信息-----------------\r\n");		
	printf("SPI Flash容量：%d KB\r\n",sFLASH_CardCapacity);	
	printf("SPI Flash剩余：%d KB\r\n",sFLASH_FreeCapacity);
}

//	函数：FatFs_FileTest
//	功能：进行文件写入和读取测试
//

uint8_t  FatFs_FileTest(void)	//文件创建和写入测试
{
	uint8_t 	i = 0;
	uint16_t BufferSize = 0;
	FIL	MyFile;			// 文件对象
	UINT 	MyFile_Num;		//	数据长度
	BYTE 	MyFile_WriteBuffer[] = "STM32 SPI Flash 文件系统测试";	//要写入的数据

	
	printf("-------------FatFs 文件创建和写入测试---------------\r\n");
	MyFile_Res = f_open(&MyFile,"1:FatFs Test.txt",FA_CREATE_ALWAYS | FA_WRITE);	//打开文件，若不存在则创建该文件
	if(MyFile_Res == FR_OK)
	{
		printf("文件打开/创建成功，准备写入数据...\r\n");
		
		MyFile_Res = f_write(&MyFile,MyFile_WriteBuffer,sizeof(MyFile_WriteBuffer),&MyFile_Num);	//向文件写入数据
		if (MyFile_Res == FR_OK)	
		{
			printf("写入成功，写入内容为：\r\n");
			printf("%s\r\n",MyFile_WriteBuffer);
		}
		else
		{
			printf("文件写入失败，请检查SPI Flash或重新格式化!\r\n");
			f_close(&MyFile);	  //关闭文件	
			return ERROR;
		}
		f_close(&MyFile);	  //关闭文件	
	}
	else
	{
		printf("无法打开/创建文件，请检查SPI Flash或重新格式化!\r\n");
		f_close(&MyFile);	  //关闭文件	
		return ERROR;		
	}

	
	printf("-------------FatFs 文件读取测试---------------\r\n");	
	
	BufferSize = sizeof(MyFile_WriteBuffer)/sizeof(BYTE);									// 计算写入的数据长度
	MyFile_Res = f_open(&MyFile,"1:FatFs Test.txt",FA_OPEN_EXISTING | FA_READ);	//打开文件，若不存在则创建该文件
	MyFile_Res = f_read(&MyFile,sFLASH_ReadBuffer,BufferSize,&MyFile_Num);			// 读取文件
	if(MyFile_Res == FR_OK)
	{
		printf("文件读取成功，正在校验数据...\r\n");
		
		for(i=0;i<BufferSize;i++)
		{
			if(MyFile_WriteBuffer[i] != sFLASH_ReadBuffer[i])		// 校验数据
			{
				printf("校验失败，请检查SPI Flash或重新格式化!\r\n");
				f_close(&MyFile);	  //关闭文件	
				return ERROR;
			}
		}
		printf("校验成功，读出的数据为：\r\n");
		printf("%s\r\n",sFLASH_ReadBuffer);
	}	
	else
	{
		printf("无法读取文件，请检查SPI Flash或重新格式化!\r\n");
		f_close(&MyFile);	  //关闭文件	
		return ERROR;		
	}	
	
	f_close(&MyFile);	  //关闭文件	
	return SUCCESS;
}


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
	HAL_Init();				// 初始化HAL库
	SystemClock_Config();	// 配置系统时钟，主频550MHz
	LED_Init();				// 初始化LED引脚
	USART1_Init();			// USART1初始化	

	FatFs_Check();			//判断FatFs是否挂载成功，若没有创建FatFs则格式化SD卡
	FatFs_GetVolume();	    // 计算设备容量
	FatFs_FileTest();		//文件创建和写入测试
	
	while (1)
	{
		LED1_Toggle;
		HAL_Delay(100);
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
  
 
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI;    
    
	/* 为了设置方便，将 高速时钟HCLK3 作为OSPI的内核时钟，速度为275M，再经过2分频得到 137.5M 的驱动时钟  */  
	/* 虽然W25Q64JV 所允许的最高时钟为 133M */
	/* 但实际测试中，将驱动时钟拉到将近200M，w25q64还是可以正常读写，所以只超出4.5M的频率不用担心稳定性*/
    /* 若用户使用厂家严苛，可以重新配置内核时钟 */
 	PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_D1HCLK; 
  
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

