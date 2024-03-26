#include "main.h"

int main(void)
{
#ifdef MCU
	lw_init_borad();//STM32板子初始化 - STM32 board initialization
#endif	

	float* model_out = lw_model(); //运行模型 - Run the model

	printf("model out = [%f %f %f %f]\r\n", model_out[0], model_out[1], model_out[2], model_out[3]);
	
	
//	printf("model out = [%f %f %f %f %f %f %f]\r\n", model_out[0], model_out[1], model_out[2], model_out[3], model_out[4], model_out[5], model_out[6]);

	free(model_out);

	return 0;
}











