/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*  @date    2023-4-10
	*	@author  ���ͿƼ�
   ************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32H723ZGT6���İ� ���ͺţ�FK723M1-ZGT6��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> ����˵����
	*
	*	1.����LED��ʹ��HAL���Դ�����ʱ����ʵ����˸
	*   2.��ֲFatFs�����м򵥵Ĳ���
	*	3.OSPI���ģʽ��ʹ�ÿ⺯��ֱ�Ӷ�д��
	*	4.Ĭ������QSPI����ʱ��Ϊ137.5M
	*
>>>>> ���ڴ�ӡ˵����
	*
	*	USART1ʹ�õ���PA9/PA10�����ڲ�����115200
	*	
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "usart.h"
#include "ff.h"

/********************************************** �������� *******************************************/

FATFS 	sFLASH_FatFs; 					// �ļ�ϵͳ����
FRESULT 	MyFile_Res;       			// ������� 
BYTE 		sFLASH_ReadBuffer[1024];	//Ҫ����������


//	������FatFs_Check
//	���ܣ�����FatFs�ļ�ϵͳ�Ĺ���
//
void FatFs_Check(void)	//�ж�FatFs�Ƿ���سɹ�����û�д���FatFs���ʽ��SD��
{
	BYTE work[FF_MAX_SS]; 
	
	MyFile_Res = f_mount(&sFLASH_FatFs,"1:",1);	//��ʼ��SPI Flash��������FatFs
	
	if (MyFile_Res == FR_OK)	//�ж��Ƿ���سɹ�
	{
		printf("\r\nSPI Flash�ļ�ϵͳ���سɹ�\r\n");
	}
	else		
	{
		printf("SPI Flash��δ�����ļ�ϵͳ��������ʽ��\r\n");
		
		MyFile_Res = f_mkfs("1:",FM_FAT,0,work,sizeof work);		//��ʽ����FAT32����Ĭ�ϴ�С16K
		
		if (MyFile_Res == FR_OK)		//�ж��Ƿ��ʽ���ɹ�
			printf("SPI Flash��ʽ���ɹ���\r\n");
		else
			printf("��ʽ��ʧ�ܣ���������SPI Flash��\r\n");
	}
}

//	������FatFs_GetVolume
//	���ܣ������豸��������������������ʣ������
//

void  FatFs_GetVolume(void)	// �����豸����
{
	FATFS *fs;		//����ṹ��ָ��
	uint32_t sFLASH_CardCapacity = 0;		//SD����������
	uint32_t sFLASH_FreeCapacity = 0;		//SD����������
	DWORD fre_clust, fre_sect, tot_sect; 	//���дأ���������������������

	f_getfree("1:",&fre_clust,&fs);			//��ȡ�豸ʣ��Ĵ�

	tot_sect = (fs->n_fatent-2) * fs->csize;	//���������� = �ܵĴ� * ÿ���ذ�����������
	fre_sect = fre_clust * fs->csize;			//����ʣ��Ŀ���������	   

	sFLASH_CardCapacity = tot_sect * 4096 / 1024;	//������ = �������� * ÿ�������ֽ��� / 1K
	sFLASH_FreeCapacity = fre_sect * 4096 / 1024;	//����ʣ�����������λΪKB

	printf("-------------------��ȡ�豸������Ϣ-----------------\r\n");		
	printf("SPI Flash������%d KB\r\n",sFLASH_CardCapacity);	
	printf("SPI Flashʣ�ࣺ%d KB\r\n",sFLASH_FreeCapacity);
}

//	������FatFs_FileTest
//	���ܣ������ļ�д��Ͷ�ȡ����
//

uint8_t  FatFs_FileTest(void)	//�ļ�������д�����
{
	uint8_t 	i = 0;
	uint16_t BufferSize = 0;
	FIL	MyFile;			// �ļ�����
	UINT 	MyFile_Num;		//	���ݳ���
	BYTE 	MyFile_WriteBuffer[] = "STM32 SPI Flash �ļ�ϵͳ����";	//Ҫд�������

	
	printf("-------------FatFs �ļ�������д�����---------------\r\n");
	MyFile_Res = f_open(&MyFile,"1:FatFs Test.txt",FA_CREATE_ALWAYS | FA_WRITE);	//���ļ������������򴴽����ļ�
	if(MyFile_Res == FR_OK)
	{
		printf("�ļ���/�����ɹ���׼��д������...\r\n");
		
		MyFile_Res = f_write(&MyFile,MyFile_WriteBuffer,sizeof(MyFile_WriteBuffer),&MyFile_Num);	//���ļ�д������
		if (MyFile_Res == FR_OK)	
		{
			printf("д��ɹ���д������Ϊ��\r\n");
			printf("%s\r\n",MyFile_WriteBuffer);
		}
		else
		{
			printf("�ļ�д��ʧ�ܣ�����SPI Flash�����¸�ʽ��!\r\n");
			f_close(&MyFile);	  //�ر��ļ�	
			return ERROR;
		}
		f_close(&MyFile);	  //�ر��ļ�	
	}
	else
	{
		printf("�޷���/�����ļ�������SPI Flash�����¸�ʽ��!\r\n");
		f_close(&MyFile);	  //�ر��ļ�	
		return ERROR;		
	}

	
	printf("-------------FatFs �ļ���ȡ����---------------\r\n");	
	
	BufferSize = sizeof(MyFile_WriteBuffer)/sizeof(BYTE);									// ����д������ݳ���
	MyFile_Res = f_open(&MyFile,"1:FatFs Test.txt",FA_OPEN_EXISTING | FA_READ);	//���ļ������������򴴽����ļ�
	MyFile_Res = f_read(&MyFile,sFLASH_ReadBuffer,BufferSize,&MyFile_Num);			// ��ȡ�ļ�
	if(MyFile_Res == FR_OK)
	{
		printf("�ļ���ȡ�ɹ�������У������...\r\n");
		
		for(i=0;i<BufferSize;i++)
		{
			if(MyFile_WriteBuffer[i] != sFLASH_ReadBuffer[i])		// У������
			{
				printf("У��ʧ�ܣ�����SPI Flash�����¸�ʽ��!\r\n");
				f_close(&MyFile);	  //�ر��ļ�	
				return ERROR;
			}
		}
		printf("У��ɹ�������������Ϊ��\r\n");
		printf("%s\r\n",sFLASH_ReadBuffer);
	}	
	else
	{
		printf("�޷���ȡ�ļ�������SPI Flash�����¸�ʽ��!\r\n");
		f_close(&MyFile);	  //�ر��ļ�	
		return ERROR;		
	}	
	
	f_close(&MyFile);	  //�ر��ļ�	
	return SUCCESS;
}


/********************************************** �������� *******************************************/

void SystemClock_Config(void);		// ʱ�ӳ�ʼ��

/***************************************************************************************************
*	�� �� ��: main
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: ����������
*	˵    ��: ��
****************************************************************************************************/

int main(void)
{
	SCB_EnableICache();		// ʹ��ICache
	SCB_EnableDCache();		// ʹ��DCache
	HAL_Init();				// ��ʼ��HAL��
	SystemClock_Config();	// ����ϵͳʱ�ӣ���Ƶ550MHz
	LED_Init();				// ��ʼ��LED����
	USART1_Init();			// USART1��ʼ��	

	FatFs_Check();			//�ж�FatFs�Ƿ���سɹ�����û�д���FatFs���ʽ��SD��
	FatFs_GetVolume();	    // �����豸����
	FatFs_FileTest();		//�ļ�������д�����
	
	while (1)
	{
		LED1_Toggle;
		HAL_Delay(100);
	}
}


/****************************************************************************************************/
/**
  *   ϵͳʱ�����ã�
  *
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 550000000 (CPU ��Ƶ 550MHz)
  *            HCLK(Hz)                       = 275000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 1 (AHB  Clock  275 MHz)
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  137.5MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  137.5MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  137.5MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  137.5MHz)
  *            HSE Frequency(Hz)              = 25000000  (�ⲿ����Ƶ��)
  *            PLL_M                          = 10
  *            PLL_N                          = 220
  *            PLL_P                          = 1
  *
  *				CPU��Ƶ = HSE Frequency / PLL_M * PLL_N / PLL_P = 25M /10*220/1 = 550M
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
    
	/* Ϊ�����÷��㣬�� ����ʱ��HCLK3 ��ΪOSPI���ں�ʱ�ӣ��ٶ�Ϊ275M���پ���2��Ƶ�õ� 137.5M ������ʱ��  */  
	/* ��ȻW25Q64JV ����������ʱ��Ϊ 133M */
	/* ��ʵ�ʲ����У�������ʱ����������200M��w25q64���ǿ���������д������ֻ����4.5M��Ƶ�ʲ��õ����ȶ���*/
    /* ���û�ʹ�ó����Ͽ����������������ں�ʱ�� */
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

