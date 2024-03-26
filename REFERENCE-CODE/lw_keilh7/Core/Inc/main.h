#ifndef __MIAN__
#define __MIAN__

	#include <stdio.h>
	#include <stdint.h>
	#include <stdlib.h>
	#include <math.h>
	#include <string.h>
	#include "lw_cnn.h"
	#include "lw_stft.h"
	#include "stm32h723xx.h" //记得包含这个库, 里面定义了__FPU_PRESENT,必须放在#include "arm_math.h"前面 - remember to include this library, which defines __FPU_PRESENT, and must be placed before #include "arm_math.h"
	#include "arm_math.h" //DSP库,FFT需要 - DSP library, required for FFT


	//任意时刻只二选一个宏 - only two macros can be selected at any time
	#define MCU
//  #define SD

#ifdef MCU
	#include "stm32h7xx_hal.h"
	#include "lw_init.h"
	#include "lw_usart.h"
#endif

#ifdef SD
	#include "stm32h7xx_hal.h"
	#include "lw_init.h"
	#include "lw_usart.h"
#endif

	#define FLASH_DATA_BASE_ADDR	0x08020000U //用于存放参数的FLASH起始地址, 前64K=0x10000用于存放代码, stm32h7起始地址必须按照Flash扇区起始位置开始 - FLASH start address used to store parameters, the first 64K=0x10000 is used to store code, and the start address of stm32h7 must start from the start of the Flash sector

#endif

