/**
 * @file main.c
 * @author SHUAIWEN CUI (SHUAIWEN001@e.ntu.edu.sg)
 * @brief This project incorporates basic function modules for sensor prototyping, including LED, USART, SDMMC, FATFS, CMSIS DSP and CMSIS NN. 
 * ! LED - LED can be used to indicate the running status of the program.
 * ! USART - USART can be used to print the running status of the program.
 * ! SDMMC - SDMMC can be used to read and write data to the SD card.
 * ! FATFS - FATFS can be used to manage the files on memory and SD card.
 * ! CMSIS DSP - CMSIS DSP can be used to perform digital signal processing operations.
 * ! CMSIS NN - CMSIS NN can be used to perform basic neural network operations.
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

// SYSTEM MODULES
#ifdef _RTE_
#include "RTE_Components.h"
#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif
#endif

// DSP MODULES
#include "arm_math.h"

// NN MODULES
#include "arm_nn_tables.h"
#include "arm_nnfunctions.h"
#include "arm_nnsupportfunctions.h"

// NN EXAMPLES
#include "arm_nnexamples_cifar10_parameter.h"
#include "arm_nnexamples_cifar10_weights.h"
#include "arm_nnexamples_cifar10_inputs.h"

/* DEPENDENCIES ----------------------------------------------- START*/


/* STATEMENTS ------------------------------------------------- START*/

// SDMMC/SD Card
#define NumOf_Blocks 64
#define Test_BlockSize ((BLOCKSIZE * NumOf_Blocks) >> 2) // Define the data size, the SD block size is 512 bytes, because it is a 32-bit array, so divide by 4
#define Test_Addr 0x00

// DSP 
#define FFT_LENGTH 1024 // FFT长度，默认是1024点FFT

// FATFS / SD Card
FATFS SD_FatFs;		// file system object
FRESULT MyFile_Res; // operation result
char SDPath[4];		// SD card logical drive path

// DSP
float fft_inputbuf[FFT_LENGTH * 2]; // FFT Input array
float fft_outputbuf[FFT_LENGTH];	// FFT Output array


// NN
// include the input and weights
static q7_t conv1_wt[CONV1_IM_CH * CONV1_KER_DIM * CONV1_KER_DIM * CONV1_OUT_CH] = CONV1_WT;
static q7_t conv1_bias[CONV1_OUT_CH] = CONV1_BIAS;

static q7_t conv2_wt[CONV2_IM_CH * CONV2_KER_DIM * CONV2_KER_DIM * CONV2_OUT_CH] = CONV2_WT;
static q7_t conv2_bias[CONV2_OUT_CH] = CONV2_BIAS;

static q7_t conv3_wt[CONV3_IM_CH * CONV3_KER_DIM * CONV3_KER_DIM * CONV3_OUT_CH] = CONV3_WT;
static q7_t conv3_bias[CONV3_OUT_CH] = CONV3_BIAS;

static q7_t ip1_wt[IP1_DIM * IP1_OUT] = IP1_WT;
static q7_t ip1_bias[IP1_OUT] = IP1_BIAS;

/* Here the image_data should be the raw uint8 type RGB image in [RGB, RGB, RGB ... RGB] format */
uint8_t image_data[CONV1_IM_CH * CONV1_IM_DIM * CONV1_IM_DIM] = IMG_DATA;
q7_t output_data[IP1_OUT];

// vector buffer: max(im2col buffer,average pool buffer, fully connected buffer)
q7_t col_buffer[2 * 5 * 5 * 32 * 2];

q7_t scratch_buffer[32 * 32 * 10 * 4];

/* STATEMENTS --------------------------------------------------- END*/

/* HELPER FUNCTIONS ------------------------------------------- START*/

/**
 * @name FatFs_Check
 * @brief check if the FatFs file system is mounted, if not, format the SD card
 */
void FatFs_Check(void) // check if the FatFs file system is mounted, if not, format the SD card
{
	BYTE work[FF_MAX_SS];

	FATFS_LinkDriver(&SD_Driver, SDPath);	  // initialize the SD Card driver
	MyFile_Res = f_mount(&SD_FatFs, "0:", 1); // mount the default drive

	if (MyFile_Res == FR_OK) // check if the FatFs file system is mounted
	{
		printf("\r\nSD Card FATFS system is mounted successfully!\r\n");
	}
	else
	{
		printf("File system on SD card is not created yet, about to format the SD card...\r\n");

		MyFile_Res = f_mkfs("0:", FM_FAT32, 0, work, sizeof work); // format the SD card, FAT32, cluster size by default 16K

		if (MyFile_Res == FR_OK) // check if the SD card is formatted successfully
			printf("SD card is formatted successfully\r\n");
		else
			printf("SD card format failed, please check the SD card or reformat it!\r\n");
	}
}

/**
 * @name FatFs_GetVolume
 * @brief calculate the capacity of the device, including the total capacity and the remaining capacity
 */
void FatFs_GetVolume(void) // compute the capacity of the device
{
	FATFS *fs;							 // define the structure pointer
	uint32_t SD_CardCapacity = 0;		 // SD card total capacity
	uint32_t SD_FreeCapacity = 0;		 // SD card free capacity
	DWORD fre_clust, fre_sect, tot_sect; // free cluster, free sector number, total sector number

	f_getfree("0:", &fre_clust, &fs); // get the free cluster of the SD card

	tot_sect = (fs->n_fatent - 2) * fs->csize; // total sector number = total cluster number * sector number per cluster
	fre_sect = fre_clust * fs->csize;		   // compute the remaining available sector number

	SD_CardCapacity = tot_sect / 2048; // SD card total capacity = total sector number * 512 (bytes per sector) / 1048576 (converted to MB)
	SD_FreeCapacity = fre_sect / 2048; // compute the remaining capacity, unit is M
	printf("-------------------get device volume info-----------------\r\n");
	printf("SD capacity: %dMB\r\n", SD_CardCapacity);
	printf("SD remaining capacity: %dMB\r\n", SD_FreeCapacity);
}

/**
 * @name FatFs_FileTest
 * @brief file IO test (writing and reading)
 * 
 */
uint8_t FatFs_FileTest(void) // create and write file test
{
	uint8_t i = 0;
	uint16_t BufferSize = 0;
	FIL MyFile;											                 // file object
	UINT MyFile_Num;									                 // data length
	BYTE MyFile_WriteBuffer[] = "STM32 SD Card File System Test Text";   // data to be written
	BYTE MyFile_ReadBuffer[1024];						                 // data to be read

	printf("-------------FatFs file creating and writing test--------------\r\n");

	MyFile_Res = f_open(&MyFile, "0:FatFs Test.txt", FA_CREATE_ALWAYS | FA_WRITE); // open file, create if not exist
	if (MyFile_Res == FR_OK)
	{
		printf("file opened / created successfully, ready to write in data...\r\n");

		MyFile_Res = f_write(&MyFile, MyFile_WriteBuffer, sizeof(MyFile_WriteBuffer), &MyFile_Num); // write data to file
		if (MyFile_Res == FR_OK)
		{
			printf("file writing succeeded, the data wrote in:\r\n");
			printf("%s\r\n", MyFile_WriteBuffer);
		}
		else
		{
			printf("file writing failed, please check SD card or reformat it!\r\n");
			f_close(&MyFile); // close file
			return ERROR;
		}
		f_close(&MyFile); // close file
	}
	else
	{
		printf(" file open / create failed, please check SD card or reformat it!\r\n");
		f_close(&MyFile); // close file
		return ERROR;
	}

	printf("-------------FatFs file reading test---------------\r\n");

	BufferSize = sizeof(MyFile_WriteBuffer) / sizeof(BYTE);						  // compute the length of the written data
	MyFile_Res = f_open(&MyFile, "0:FatFs Test.txt", FA_OPEN_EXISTING | FA_READ); // open file, create if not exist
	MyFile_Res = f_read(&MyFile, MyFile_ReadBuffer, BufferSize, &MyFile_Num);	  // read file
	if (MyFile_Res == FR_OK)
	{
		printf("file reading succeeded, verifying data...\r\n");

		for (i = 0; i < BufferSize; i++)
		{
			if (MyFile_WriteBuffer[i] != MyFile_ReadBuffer[i]) // verify the data
			{
				printf("data verification failed, please check SD card or reformat it!\r\n");
				f_close(&MyFile); // close file
				return ERROR;
			}
		}
		printf("data verification succeeded, the data read out:\r\n");
		printf("%s\r\n", MyFile_ReadBuffer);
	}
	else
	{
		printf("file reading failed, please check SD card or reformat it!\r\n");
		f_close(&MyFile); // close file
		return ERROR;
	}

	f_close(&MyFile); // close file
	return SUCCESS;
}

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
	// state variables
	int i;
	arm_cfft_radix4_instance_f32 scfft;

	// initialize the system
	SCB_EnableICache();	  // enable ICache
	SCB_EnableDCache();	  // enable DCache
	HAL_Init();			  // init HAL library
	SystemClock_Config(); // configure system clock, main frequency 550MHz
	LED_Init();			  // init LED pin
	USART1_Init();		  // init USART1

	// <I SD CARD / FATFS TEST>
	printf("\r\n[PART I: SD CARD & FATFS TEST]\r\n");
	FatFs_Check();	   // check if the FatFs file system is mounted, if not, format the SD card
	FatFs_GetVolume(); // compute the capacity of the device
	FatFs_FileTest();  // file IO test (writing and reading)

	// <II CMSIS DSP TEST>
	printf("\r\n[PART II: CMSIS DSP TEST - FFT]\r\n");
	arm_cfft_radix4_init_f32(&scfft, FFT_LENGTH, 0, 1); // init the scfft structure, set FFT related parameters

	for (i = 0; i < FFT_LENGTH; i++) // generate signal sequence
	{
		fft_inputbuf[2 * i] = 100 +
							  10 * arm_sin_f32(2 * PI * i / FFT_LENGTH) +
							  30 * arm_sin_f32(2 * PI * i * 4 / FFT_LENGTH) +
							  50 * arm_cos_f32(2 * PI * i * 8 / FFT_LENGTH); // generate input signal real part
		fft_inputbuf[2 * i + 1] = 0;										 // imaginary part is all 0
	}

	arm_cfft_radix4_f32(&scfft, fft_inputbuf); // FFT calculation (base 4)

	arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH); // get the amplitude value of the complex modulus of the operation result

	printf("FFT Result:\r\n");
	for (i = 0; i < FFT_LENGTH; i++)
	{
		printf("fft_outputbuf[%d]:%f\r\n", i, fft_outputbuf[i]);
	}

#ifdef RTE_Compiler_EventRecorder
	EventRecorderInitialize(EventRecordAll, 1); // initialize and start Event Recorder
#endif

	// <II CMSIS NN TEST>
	printf("\r\n[PART III: CMSIS NN TEST - CNN FOR IMAGE CLASSIFICATION]\r\n");
	printf("start execution\n");
	/* start the execution */

	q7_t *img_buffer1 = scratch_buffer;
	q7_t *img_buffer2 = img_buffer1 + 32 * 32 * 32;

	/* input pre-processing */
	int mean_data[3] = INPUT_MEAN_SHIFT;
	unsigned int scale_data[3] = INPUT_RIGHT_SHIFT;
	for (int i = 0; i < 32 * 32 * 3; i += 3)
	{
		img_buffer2[i] = (q7_t)__SSAT(((((int)image_data[i] - mean_data[0]) << 7) + (0x1 << (scale_data[0] - 1))) >> scale_data[0], 8);
		img_buffer2[i + 1] = (q7_t)__SSAT(((((int)image_data[i + 1] - mean_data[1]) << 7) + (0x1 << (scale_data[1] - 1))) >> scale_data[1], 8);
		img_buffer2[i + 2] = (q7_t)__SSAT(((((int)image_data[i + 2] - mean_data[2]) << 7) + (0x1 << (scale_data[2] - 1))) >> scale_data[2], 8);
	}

	// conv1 img_buffer2 -> img_buffer1
	arm_convolve_HWC_q7_RGB(img_buffer2, CONV1_IM_DIM, CONV1_IM_CH, conv1_wt, CONV1_OUT_CH, CONV1_KER_DIM, CONV1_PADDING,
							CONV1_STRIDE, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_OUT_RSHIFT, img_buffer1, CONV1_OUT_DIM,
							(q15_t *)col_buffer, NULL);

	arm_relu_q7(img_buffer1, CONV1_OUT_DIM * CONV1_OUT_DIM * CONV1_OUT_CH);

	// pool1 img_buffer1 -> img_buffer2
	arm_maxpool_q7_HWC(img_buffer1, CONV1_OUT_DIM, CONV1_OUT_CH, POOL1_KER_DIM,
					   POOL1_PADDING, POOL1_STRIDE, POOL1_OUT_DIM, NULL, img_buffer2);

	// conv2 img_buffer2 -> img_buffer1
	arm_convolve_HWC_q7_fast(img_buffer2, CONV2_IM_DIM, CONV2_IM_CH, conv2_wt, CONV2_OUT_CH, CONV2_KER_DIM,
							 CONV2_PADDING, CONV2_STRIDE, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, img_buffer1,
							 CONV2_OUT_DIM, (q15_t *)col_buffer, NULL);

	arm_relu_q7(img_buffer1, CONV2_OUT_DIM * CONV2_OUT_DIM * CONV2_OUT_CH);

	// pool2 img_buffer1 -> img_buffer2
	arm_maxpool_q7_HWC(img_buffer1, CONV2_OUT_DIM, CONV2_OUT_CH, POOL2_KER_DIM,
					   POOL2_PADDING, POOL2_STRIDE, POOL2_OUT_DIM, col_buffer, img_buffer2);

	// conv3 img_buffer2 -> img_buffer1
	arm_convolve_HWC_q7_fast(img_buffer2, CONV3_IM_DIM, CONV3_IM_CH, conv3_wt, CONV3_OUT_CH, CONV3_KER_DIM,
							 CONV3_PADDING, CONV3_STRIDE, conv3_bias, CONV3_BIAS_LSHIFT, CONV3_OUT_RSHIFT, img_buffer1,
							 CONV3_OUT_DIM, (q15_t *)col_buffer, NULL);

	arm_relu_q7(img_buffer1, CONV3_OUT_DIM * CONV3_OUT_DIM * CONV3_OUT_CH);

	// pool3 img_buffer-> img_buffer2
	arm_maxpool_q7_HWC(img_buffer1, CONV3_OUT_DIM, CONV3_OUT_CH, POOL3_KER_DIM,
					   POOL3_PADDING, POOL3_STRIDE, POOL3_OUT_DIM, col_buffer, img_buffer2);

	arm_fully_connected_q7_opt(img_buffer2, ip1_wt, IP1_DIM, IP1_OUT, IP1_BIAS_LSHIFT, IP1_OUT_RSHIFT, ip1_bias,
							   output_data, (q15_t *)img_buffer1);

	arm_softmax_q7(output_data, 10, output_data);

	for (int i = 0; i < 10; i++)
	{
		printf("%d: %d\n", i, output_data[i]);
	}

	// END OF Test
	printf("Test is all done!\r\n");

	while (1)
	{
		LED1_Toggle;
		HAL_Delay(500);
	}
}
/* MAIN FUNCTION ------------------------------------------------ END*/


