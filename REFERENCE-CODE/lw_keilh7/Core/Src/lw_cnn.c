#include "main.h"

#ifndef MCU
	char* file_name = "1_weight_bias_image.bin";
#endif

//模型输出 - Model Output
float* lw_model(void)
{
	//卷积 - Convolution Neural Network
	lw_head_t* head = lw_read_head();//文件头 - File Head
		
	float* test_data = lw_read_signal(head);//原始数据,实际采集的传感器数据 - Original Data, Actual Sensor Data
	float* stft_data = lw_stft_fft(head, test_data); //做STFT - Do STFT
	//float* stft_data = lw_stft_dft(head, test_data); //做STFT - Do STFT
	
	float* conv_out_1 = lw_conv_computing(head, stft_data, layer_1);//卷积,带偏置 - Convolution, With Bias
	float* relu_out_1 = lw_relu_computing(head, conv_out_1, layer_1);//激活 - Activation
	float* pool_out_1 = lw_pool_computing(head, relu_out_1, layer_1);//池化 - Pooling

	float* conv_out_2 = lw_conv_computing(head, pool_out_1, layer_2);
	float* relu_out_2 = lw_relu_computing(head, conv_out_2, layer_2);
	float* pool_out_2 = lw_pool_computing(head, relu_out_2, layer_2);

	float* conv_out_3 = lw_conv_computing(head, pool_out_2, layer_3);
	float* relu_out_3 = lw_relu_computing(head, conv_out_3, layer_3);
	float* pool_out_3 = lw_pool_computing(head, relu_out_3, layer_3);

	//全连接 - Full Connect
	float* linear_out = lw_line_computing(head, pool_out_3, linear_4);//线性,带偏置 - Linear, With Bias

	return linear_out;
}

//read head
lw_head_t* lw_read_head()
{
	lw_head_t* head = (lw_head_t*)calloc(sizeof(lw_head_t), 1); //malloc memory

	if (head != NULL)
	{
#ifdef MCU
		lw_read_flash((float*)head, sizeof(lw_head_t) / sizeof(float), 0);
#else
		lw_read_sd((float*)head, sizeof(lw_head_t) / sizeof(float), 0);
#endif
		//输出网络结构参数 - Output Network Structure Parameters
		printf("%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d\r\n", (uint32_t)head->con_layer_input_dimension_1, (uint32_t)head->con_layer_output_dimension_1, (uint32_t)head->con_layer_kernel_1, (uint32_t)head->con_layer_pooling_1,
			                                              (uint32_t)head->con_layer_input_dimension_2, (uint32_t)head->con_layer_output_dimension_2, (uint32_t)head->con_layer_kernel_2, (uint32_t)head->con_layer_pooling_2,
			                                              (uint32_t)head->con_layer_input_dimension_3, (uint32_t)head->con_layer_output_dimension_3, (uint32_t)head->con_layer_kernel_3, (uint32_t)head->con_layer_pooling_3);
	}
	else
	{
		printf("Calloc head fail!\r\n");
	}

	return head;
}

//read image
float* lw_read_image(lw_head_t* head)
{
	uint32_t seek = (uint32_t)(sizeof(float) * (head->head_len + head->para_size));

	float* image = (float*)calloc((uint32_t)(sizeof(float) * (head->image_size * head->image_size)), 1);

	if (image != NULL)
	{
#ifdef MCU
		lw_read_flash(image, head->image_size *head->image_size, seek);
#else 
		lw_read_sd(image, head->image_size *head->image_size, seek);
#endif 
	}
	else
	{
		printf("Calloc image fail!\r\n");
	}

	return image;
}


//read original signal
float* lw_read_signal(lw_head_t* head)
{
	uint32_t seek = (uint32_t)(sizeof(float) * (head->head_len + head->para_size));

	uint32_t signal_input_len = (head->image_size - 1) * ((head->image_size *2)/2) + (head->image_size *2); //做一张图片STFT的信号长度,计算公式参考mycnn.py 256行左右 - The length of the signal to do STFT for a picture, the calculation formula refers to mycnn.py around line 256
	float *signal = (float*)calloc(signal_input_len * sizeof(float), 1); //stft输入数据 - stft input data
	if (signal != NULL)
	{
#ifdef MCU
		lw_read_flash(signal, signal_input_len, seek);
#else 
		lw_read_sd(signal, head->image_size *head->image_size, seek);
#endif 
	}
	else
	{
		printf("Calloc image fail!\r\n");
	}

	return signal;
}

// read weight
float* lw_read_weight(lw_head_t* head, lw_layer_t layer)
{
	uint32_t seek = 0; //offset
	uint32_t weight_size = 0; //para size


	// judge layer
	switch (layer)
	{
			//读取一层卷积的全部权重数据,不是一个filter的 - Read all weight data of a convolution layer, not a filter
		case layer_1:
			weight_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1));
			seek = (uint32_t)(sizeof(float) * head->head_len);
			break;

		case layer_2:
			weight_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_2 * head->con_layer_input_dimension_2 * head->con_layer_kernel_2 * head->con_layer_kernel_2));
			seek = (uint32_t)(sizeof(float) * (head->head_len + head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1 + head->con_layer_output_dimension_1));
			break;

		case layer_3:
			weight_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_3 * head->con_layer_input_dimension_3 * head->con_layer_kernel_3 * head->con_layer_kernel_3));
			seek = (uint32_t)(sizeof(float) * (head->head_len + head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1 + head->con_layer_output_dimension_1 +
				head->con_layer_output_dimension_2 * head->con_layer_input_dimension_2 * head->con_layer_kernel_2 * head->con_layer_kernel_2 + head->con_layer_output_dimension_2));
			break;

		case linear_4:
			weight_size = (uint32_t)(sizeof(float) * (head->full_connect_len * head->classification_out));
			seek = (uint32_t)(sizeof(float) * (head->head_len + head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1 + head->con_layer_output_dimension_1 +
				head->con_layer_output_dimension_2 * head->con_layer_input_dimension_2 * head->con_layer_kernel_2 * head->con_layer_kernel_2 + head->con_layer_output_dimension_2 +
				head->con_layer_output_dimension_3 * head->con_layer_input_dimension_3 * head->con_layer_kernel_3 * head->con_layer_kernel_3 + head->con_layer_output_dimension_3));
			break;
	}

	float* weight_data = (float*)calloc(weight_size, 1);
	if (weight_data != NULL)
	{
#ifdef MCU
		lw_read_flash(weight_data, weight_size/sizeof(float), seek);
#else
		lw_read_sd(weight_data, weight_size/sizeof(float), seek);
#endif 
	}

	return weight_data;
}

// read weight
float* lw_read_bias(lw_head_t* head, lw_layer_t layer)
{
	uint32_t seek = 0; //offset
	uint32_t bias_size = 0; //bias size
	uint32_t wieght_size = 0; //weight size

	switch (layer)
	{
		case layer_1:
			bias_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_1));
			wieght_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1));
			seek = wieght_size + (uint32_t)(sizeof(float) * head->head_len);
			break;

		case layer_2:
			bias_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_2));
			wieght_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_2 * head->con_layer_input_dimension_2 * head->con_layer_kernel_2 * head->con_layer_kernel_2));
			seek = wieght_size + (uint32_t)(sizeof(float) * (head->head_len + head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1 + head->con_layer_output_dimension_1));
			break;

		case layer_3:
			bias_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_3));
			wieght_size = (uint32_t)(sizeof(float) * (head->con_layer_output_dimension_3 * head->con_layer_input_dimension_3 * head->con_layer_kernel_3 * head->con_layer_kernel_3));
			seek = wieght_size + (uint32_t)(sizeof(float) * (head->head_len + head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1 + head->con_layer_output_dimension_1 +
				head->con_layer_output_dimension_2 * head->con_layer_input_dimension_2 * head->con_layer_kernel_2 * head->con_layer_kernel_2 + head->con_layer_output_dimension_2));
			break;

		case linear_4:
			bias_size = (uint32_t)(sizeof(float) * (head->classification_out));
			wieght_size = (uint32_t)(sizeof(float) * (head->full_connect_len * head->classification_out));
			seek = wieght_size + (uint32_t)(sizeof(float) * (head->head_len + head->con_layer_output_dimension_1 * head->con_layer_input_dimension_1 * head->con_layer_kernel_1 * head->con_layer_kernel_1 + head->con_layer_output_dimension_1 +
				head->con_layer_output_dimension_2 * head->con_layer_input_dimension_2 * head->con_layer_kernel_2 * head->con_layer_kernel_2 + head->con_layer_output_dimension_2 +
				head->con_layer_output_dimension_3 * head->con_layer_input_dimension_3 * head->con_layer_kernel_3 * head->con_layer_kernel_3 + head->con_layer_output_dimension_3));
			break;
	}

	float* bias = (float*)calloc(bias_size, 1);
	if (bias != NULL)
	{
#ifdef MCU
		lw_read_flash(bias, bias_size/sizeof(float), seek);
#else
		lw_read_sd(bias, bias_size/sizeof(float), seek);
#endif 
	}
	else
	{
		printf("Malloc bias_out fail!\r\n");
	}

	return bias;
}

//一层卷积的计算 - Convolution Computing for One Layer
float* lw_conv_computing(lw_head_t* head, float* input_images, lw_layer_t layer)
{
	//输入为一层卷积的图像和一层卷积的权重 - the input is the image of a convolution layer and the weight of a convolution layer
	uint16_t filter_nums = 0; //一层卷积总共有多少个滤波器 - How many filters are there in a convolution layer
	uint16_t channel_nums = 0; //一个filter总有几个通道 - How many channels does a filter have
	uint8_t kernel_size = 0;//卷积核大小 - Convolution kernel size

	switch (layer)
	{
	case layer_1:
		filter_nums = (uint32_t)head->con_layer_output_dimension_1;
		channel_nums = (uint32_t)head->con_layer_input_dimension_1;
		kernel_size = (uint8_t)head->con_layer_kernel_1;
		break;

	case layer_2:
		filter_nums = (uint32_t)head->con_layer_output_dimension_2;
		channel_nums = (uint32_t)head->con_layer_input_dimension_2;
		kernel_size = (uint8_t)head->con_layer_kernel_2;
		break;

	case layer_3:
		filter_nums = (uint32_t)head->con_layer_output_dimension_3;
		channel_nums = (uint32_t)head->con_layer_input_dimension_3;
		kernel_size = (uint8_t)head->con_layer_kernel_3;
		break;
	
	case linear_4:
		break;
	}

	float* conv_weights = lw_read_weight(head, layer);//读取权重参数 - Read weight parameters
	float* bias = lw_read_bias(head, layer);//读取偏置 - Read bias

	//一个filter输出图像的大小[32, 9, 9] - The size of the output image of a filter [32, 9, 9]
	uint32_t output_image_size = lw_conv_pool_output_size(head, layer, conv);
	uint32_t filters_images_size = (uint32_t)(sizeof(float) * filter_nums * output_image_size * output_image_size);
	//为组合申请内存 - Apply for memory for combination
	float* output_filters_images = (float*)calloc(filters_images_size, 1);
	if (output_filters_images != NULL)
	{
		for (uint16_t i = 0; i < filter_nums; i++)//一个filter的卷积计算 - Convolution calculation of a filter
		{
			//一个filter权重 - A filter weight
			uint32_t weight_size = (uint32_t)(sizeof(float) * channel_nums * kernel_size * kernel_size);//一个filter权重参数的大小[*, 16, 2, 2] - The size of a filter weight parameter [*, 16, 2, 2]
			float* filter_weight = (float*)calloc(weight_size, 1);//为一个filter大小的权重申请内存 - Apply for memory for a filter size weight
			memmove(filter_weight, conv_weights + (i * weight_size / sizeof(float)), weight_size);//截取权重数据作为输入, float指针加1，地址加4,所以除以sizeof(float) - Intercept the weight data as input, the float pointer plus 1, the address plus 4, so divide by sizeof(float)

			//一个filter卷积 - Convolution of a filter
			float* convd_filter_image = lw_filter_conv_computing(head, input_images, filter_weight, layer);//卷积后的filter图像,计算1个filter的卷积,输入图像[16, 10, 10]， 输入权重[16,2,2]， 输出[9,9] - Convolution of the filter image, calculate the convolution of 1 filter, input image [16, 10, 10], input weight [16,2,2], output [9,9]

			//一个filter加上偏置 - A filter plus bias
			for (uint32_t j = 0; j < output_image_size * output_image_size; j++)
			{
				convd_filter_image[j] += bias[i];
			}

			//一层卷积图像组合 - Combination of convolution images
			uint32_t image_mem_size = (uint32_t)(sizeof(float) * output_image_size * output_image_size);//一张输出图像大小[9,9] - The size of an output image [9,9]
			memmove(output_filters_images + (i * image_mem_size / sizeof(float)), convd_filter_image, image_mem_size); //组合各个filter图像数据 - Combine the image data of each filter

			free(filter_weight);
			free(convd_filter_image);
		}
		free(conv_weights);
		free(bias);
	}
	else
	{
		printf("Calloc output_filters_images fail!\r\n");
	}
	free(input_images);

	return output_filters_images;
}

//一个filter的卷积计算 - Convolution calculation of a filter
float* lw_filter_conv_computing(lw_head_t* head, float* input_images, float* filter_weights, lw_layer_t layer)
{
	//输入为一个filter的图像和一个filter的权重, 输入图像[16, 10, 10]， 输入权重[16,2,2]， 输出[9,9] - The input is the image of a filter and the weight of a filter, the input image [16, 10, 10], the input weight [16,2,2], the output [9,9]
	uint16_t channel_nums = 0; //输入图像总有几个通道 - How many channels are there in the input image
	uint8_t kernel_size = 0;//卷积核大小 - Convolution kernel size

	switch (layer)
	{
	case layer_1:
		channel_nums = (uint16_t)head->con_layer_input_dimension_1;
		kernel_size = (uint8_t)head->con_layer_kernel_1;
		break;

	case layer_2:
		channel_nums = (uint16_t)head->con_layer_input_dimension_2;
		kernel_size = (uint8_t)head->con_layer_kernel_2;
		break;

	case layer_3:
		channel_nums = (uint16_t)head->con_layer_input_dimension_3;
		kernel_size = (uint8_t)head->con_layer_kernel_3;
		break;
	
	case linear_4:
		break;
	}

	//输出特征图,所有通道的 - Output feature map, all channels
	uint32_t output_image_size = lw_conv_pool_output_size(head, layer, conv);//一个channel输出图像的大小[9, 9] - The size of the output image of a channel [9, 9]
	uint32_t output_filter_image_size = (uint32_t)(sizeof(float) * output_image_size * output_image_size);
	float* output_filter_image = (float*)calloc(output_filter_image_size, 1);//为一个filter的输出图像申请内存 - Apply for memory for the output image of a filter
	if (output_filter_image != NULL)
	{
		//一个filter所有channel图像卷积 - Convolution of all channel images of a filter
		for (uint16_t i = 0; i < channel_nums; i++)
		{
			//一个channel权重 - A channel weight
			uint32_t weight_size = (uint32_t)(sizeof(float) * kernel_size * kernel_size);//一个channel权重参数的大小[*, 2, 2] - The size of a channel weight parameter [*, 2, 2]
			float* channel_weight = (float*)calloc(weight_size, 1);//为一个channel大小的权重申请内存 - Apply for memory for a channel size weight
			memmove(channel_weight, filter_weights + (i * weight_size / sizeof(float)), weight_size);//截取权重数据作为输入, float指针加1，地址加4,所以除以sizeof(float) - Intercept the weight data as input, the float pointer plus 1, the address plus 4, so divide by sizeof(float)


			//一个channel输入特征图 - A channel input feature map
			uint32_t input_image_size = lw_conv_pool_input_size(head, layer, conv); //获取一层卷积的输入图像大小 - Get the input image size of a convolution layer
			uint32_t channel_image_size = (uint32_t)(sizeof(float) * input_image_size * input_image_size); 
			float* channel_image = (float*)calloc(channel_image_size, 1);//为一个channel的输入图像申请内存	- Apply for memory for the input image of a channel
			memmove(channel_image, input_images + (i * channel_image_size / sizeof(float)), channel_image_size); //组合各个filter图像数据 - Combine the image data of each filter

			//一个channel卷积 - Convolution of a channel
			float* convd_channel_image = lw_channel_conv_computing(head, channel_image, channel_weight, layer);//卷积后的channel的图像,计算一个channel的卷积 - Convolution of the image of the channel, calculate the convolution of a channel

			// 所有channel的特征图叠加起来 - All channel feature maps are superimposed
			for (uint32_t j = 0; j < output_image_size * output_image_size; j++)
			{
				output_filter_image[j] = output_filter_image[j] + convd_channel_image[j];
			}
			free(convd_channel_image);
			free(channel_image);
			free(channel_weight);
		}
	}
	else
	{
		printf("Calloc output_filter_image fail!\r\n");
	}

	return output_filter_image;
}


//一个通道(特征图)卷积的计算 - Convolution Computing for One Channel (Feature Map)
float* lw_channel_conv_computing(lw_head_t* head, float* channel_image, float* channel_weight, lw_layer_t layer)
{
	//输入的是一个通道(特征图)的图像 - The input is an image of a channel (feature map)
	//conv
	uint8_t conv_kernel_size = 0;//卷积核大小 - Convolution kernel size
	uint32_t input_image_size = 0;//输入图像大小 - Input image size
	uint32_t output_image_size = 0;//输出图像大小 - Output image size

	// judge layer
	switch (layer)
	{
		case layer_1:
			conv_kernel_size = (uint32_t)head->con_layer_kernel_1;//卷积核大小 - Convolution kernel size
			input_image_size = (uint32_t)head->image_size; //输入图像大小 - Input image size
			output_image_size = lw_conv_pool_output_size(head, layer_1, conv); //计算输出层图像大小 - Calculate the output layer image size
			break;

		case layer_2:
			conv_kernel_size = (uint32_t)head->con_layer_kernel_2;
			input_image_size = lw_conv_pool_input_size(head, layer_2, conv);
			output_image_size = lw_conv_pool_output_size(head, layer_2, conv);
			break;

		case layer_3:
			conv_kernel_size = (uint32_t)head->con_layer_kernel_3;
			input_image_size = lw_conv_pool_input_size(head, layer_3, conv);
			output_image_size = lw_conv_pool_output_size(head, layer_3, conv);
			break;
	}

	//一张特征图做一次完整卷积 - A complete convolution is performed on a feature map
	float* output_channel_image = (float*)calloc(sizeof(float) * output_image_size * output_image_size, 1); //一个通道的输出图像 - The output image of a channel
	if (output_channel_image != NULL)
	{
		//n为实际去除最下和最右kernel size - 1的实际有效卷积次数,也是实际输出图像大小 - n is the actual effective convolution times to remove the bottom and rightmost kernel size - 1, and it is also the actual output image size
		for (uint32_t m = 0, n = 0; m < input_image_size * (input_image_size - conv_kernel_size + 1); m++) //输入图像行，- conv_kernel_size +1为去除输入图像最下面的行 - Input image row, - conv_kernel_size + 1 is to remove the bottom row of the input image
		{
			float conv_sum_value = 0; //卷积求和 - Convolution sum
			uint32_t image_n = m; //卷积图像在输入图像左上角的起点 - The starting point of the convolution image in the upper left corner of the input image

			if ((image_n % input_image_size) < (input_image_size - conv_kernel_size + 1))//去除右侧卷积核大小-1的列 - Remove the column of the right convolution kernel size - 1
			{
				//做一次卷积 - Do a convolution
				for (uint32_t weight_k = 0; weight_k < (uint32_t)(conv_kernel_size * conv_kernel_size); weight_k++)//卷积核 - Convolution kernel
				{
					conv_sum_value += channel_image[image_n] * channel_weight[weight_k];
					//printf("weight_k = %d   image_n = %d\r\n", weight_k, image_n);

					weight_k++; //为下一次循环image_n加1还是加图像大小做准备 - Prepare for the next loop image_n plus 1 or plus image size

					if (((weight_k % conv_kernel_size) != 0) || (weight_k < conv_kernel_size)) //没有超过卷积核大小或者前面几次小于卷积核大小 - Not exceeding the convolution kernel size or the first few times less than the convolution kernel size
					{
						image_n = image_n + 1;
					}
					else
					{
						//image_n = image_n + output_image_size;
						image_n = image_n + input_image_size - conv_kernel_size + 1; //转到下一行 - Go to the next line

					}
					weight_k--;//还原回去,保证每次循环weight_k只加1，因为卷积核每次只加了1 - Restore back to ensure that each loop weight_k only adds 1, because the convolution kernel only adds 1 each time
				}

				output_channel_image[n] = conv_sum_value;
				n++;
			}
		}
	}
	else
	{
		printf("Malloc output_channel_image fail!\r\n");
	}

	return output_channel_image;
}

//激活 - Activation
float* lw_relu_computing(lw_head_t* head, float* input_images, lw_layer_t layer)
{
	//输入为一层卷积的图像和一层卷积的权重 - the input is the image of a convolution layer and the weight of a convolution layer
	uint16_t filter_nums = 0; //一层卷积总共有多少个滤波器 - How many filters are there in a convolution layer

	switch (layer)
	{
	case layer_1:
		filter_nums = (uint32_t)head->con_layer_output_dimension_1;
		break;

	case layer_2:
		filter_nums = (uint32_t)head->con_layer_output_dimension_2;
		break;

	case layer_3:
		filter_nums = (uint32_t)head->con_layer_output_dimension_3;
		break;
	}

	//激活 - Activation
	uint32_t output_image_size = lw_conv_pool_output_size(head, layer, conv);
	for (uint16_t i = 0; i < filter_nums * output_image_size * output_image_size; i++)
	{
		if (input_images[i] > 0)
		{
			input_images[i] = input_images[i];
		}
		else
		{
			input_images[i] = 0;
		}
	}

	return input_images;
}


//池化 - Pooling
float* lw_pool_computing(lw_head_t* head, float* input_images, lw_layer_t layer)
{
	uint16_t channel_nums = 0; //一层卷积总共有多少个滤波器 - How many filters are there in a convolution layer

	switch (layer)
	{
	case layer_1:
		channel_nums = (uint32_t)head->con_layer_output_dimension_1;
		break;

	case layer_2:
		channel_nums = (uint32_t)head->con_layer_output_dimension_2;
		break;

	case layer_3:
		channel_nums = (uint32_t)head->con_layer_output_dimension_3;
		break;
	}

	//输出池化图像 - Output pooling image
	//一个filter输出图像的大小[32, 9, 9] - The size of the output image of a filter [32, 9, 9]
	uint32_t output_image_size = lw_conv_pool_output_size(head, layer, pool);
	uint32_t filters_images_size = (uint32_t)(sizeof(float) * channel_nums * output_image_size * output_image_size);

	float* output_filters_images = (float*)calloc(filters_images_size, 1);
	if (output_filters_images != NULL)
	{
		//一个channel输入特征图 - A channel input feature map
		uint32_t input_image_size = lw_conv_pool_input_size(head, layer, pool); //获取第一层卷积的输入图像大小 - Get the input image size of the first convolution layer
		uint32_t channel_image_size = (uint32_t)(sizeof(float) * input_image_size * input_image_size);

		//所有filter的池化计算 - Pooling calculation of all filters
		for (uint16_t i = 0; i < channel_nums; i++)
		{
			//池化输入 - Pooling input
			float* channel_image = (float*)calloc(channel_image_size, 1);//为一个channel的输入图像申请内存 - Apply for memory for the input image of a channel
			memmove(channel_image, input_images + (i * channel_image_size / sizeof(float)), channel_image_size); //组合各个filter图像数据 - Combine the image data of each filter

			//池化 - Pooling
			float* pooled_channel_image = lw_channel_pool_computing(head, channel_image, layer);

			//池化组合 - Pooling combination
			uint32_t image_mem_size = (uint32_t)(sizeof(float) * output_image_size * output_image_size);//一张输出图像大小[9,9] - The size of an output image [9,9] 
			memmove(output_filters_images + (i * image_mem_size / sizeof(float)), pooled_channel_image, image_mem_size); //组合各个filter图像数据 - Combine the image data of each filter

			free(pooled_channel_image);
			free(channel_image);
		}
	}
	else
	{
		printf("Malloc output_filters_images fail!\r\n");
	}

	free(input_images);

	return output_filters_images;
}

//一个filter(channel)的池化 - Pooling of a filter (channel)
float* lw_channel_pool_computing(lw_head_t* head, float* channel_image, lw_layer_t layer)
{
	//输入的是一个通道(特征图)的图像 - The input is an image of a channel (feature map)
	//conv
	uint8_t pool_kernel_size = 0;//卷积核大小 - Convolution kernel size
	uint32_t input_image_size = lw_conv_pool_input_size(head, layer, pool); //输入图像大小 - Input image size
	uint32_t output_image_size = lw_conv_pool_output_size(head, layer, pool); //池化后的图像大小 - Image size after pooling

	switch (layer)
	{
		case layer_1:
			pool_kernel_size = (uint32_t)head->con_layer_pooling_1;//池化核大小 - Pooling kernel size
			break;

		case layer_2:
			pool_kernel_size = (uint32_t)head->con_layer_pooling_2;
			break;

		case layer_3:
			pool_kernel_size = (uint32_t)head->con_layer_pooling_3;
			break;
	}

	uint16_t times = (uint16_t)(input_image_size / pool_kernel_size); //行或列次数 - Number of rows or columns

	float* output_pooled_channel_image = (float*)calloc(sizeof(float) * output_image_size * output_image_size, 1); //一个通道的输出图像 - The output image of a channel
	if (output_pooled_channel_image != NULL)
	{
		uint32_t t = 0;
		for (uint32_t m = 0; m < times; m++)
		{
			for (uint32_t n = 0; n < times; n++)
			{
				uint32_t image_n = m * pool_kernel_size * input_image_size + n * pool_kernel_size;//初始基点， - Initial base point
				//printf("m = %d, image_n = %d\r\n", m, image_n);

				float max_value = 0; //最大值 - Maximum value
				for (uint32_t k = 0; k < (uint32_t)(pool_kernel_size * pool_kernel_size); k++)//image_n是元素位置 - image_n is the element position
				{
					//printf("k = %d, image_n = %d\r\n", k, image_n);

					//如果当前值大于最大值，则用当前值替换最大值 - If the current value is greater than the maximum value, replace the maximum value with the current value
					if (channel_image[image_n] > max_value)
					{
						max_value = channel_image[image_n];
					}

					k++;//为下一次循环image_n加1还是加图像大小(换到下一行)做准备 - Prepare for the next loop image_n plus 1 or plus image size (switch to the next line)
					if (((k % pool_kernel_size) != 0) || (k < pool_kernel_size))//没有超过池化核大小(移到下一行也只加1)或者前面几次小于卷积核大小 - Not exceeding the pooling kernel size (only plus 1 when moving to the next line) or the first few times less than the convolution kernel size
					{
						image_n = image_n + 1;
					}
					else
					{
						image_n = image_n + input_image_size - pool_kernel_size + 1;//移到下一行， - pool_kernel_size是为了退回到下一行的开始 - Move to the next line, - pool_kernel_size is to return to the beginning of the next line
					}
					k--;//还原回去,保证每次循环k只加1 - Restore back to ensure that each loop k only adds 1
				}
				output_pooled_channel_image[t] = max_value;
				t++;
			}
		}

	}
	else
	{
		printf("Malloc output_pooled_channel_image fail!\r\n");
	}

	return output_pooled_channel_image;
}

//线性 - Linear
float* lw_line_computing(lw_head_t* head, float* input_images, lw_layer_t layer)
{
	uint16_t row = (uint16_t)head->classification_out; //行 - row
	uint16_t col = (uint16_t)head->full_connect_len;; //列 - column

	float* line_out = (float*)calloc(sizeof(float) * row, 1);
	if (line_out != NULL)
	{
		//矩阵计算 - Matrix calculation
		float* line_weights = lw_read_weight(head, layer);//读取权重参数 - Read weight parameters

		for (uint32_t i = 0; i < row; i++) //行 - row
		{
			float sum = 0; //矩阵行×列各个数字求和 - Sum of each number in the matrix row × column
			for (uint32_t j = 0; j < col; j++) //列 - column
			{
				sum += line_weights[i * col + j] * input_images[j];
			}
			line_out[i] = sum;
		}
		free(input_images);

		//加上偏置 - Add bias
		float* bias = lw_read_bias(head, layer);
		free(head);

		for (uint32_t i = 0; i < row; i++)
		{
			line_out[i] += bias[i];
		}
		free(bias);
	}
	else
	{
		printf("Malloc line_out fail!\r\n");
	}

	return line_out;
}

// conv image output size
uint16_t lw_conv_pool_output_size(lw_head_t* head, lw_layer_t layer, lw_conv_pool_t conv_pool)
{
	uint8_t conv_kernel_size = 0;
	uint8_t pool_kernel_size = 0;

	uint16_t output_image_size = (uint8_t)head->image_size;

	for (lw_layer_t i = (lw_layer_t)1; i < (lw_layer_t)(layer + 1); i++)
	{
		// judge layer
		switch ((lw_layer_t)i)
		{
			case layer_1:
				conv_kernel_size = (uint8_t)head->con_layer_kernel_1;
				pool_kernel_size = (uint8_t)head->con_layer_pooling_1;
				break;

			case layer_2:
				conv_kernel_size = (uint8_t)head->con_layer_kernel_2;
				pool_kernel_size = (uint8_t)head->con_layer_pooling_2;
				break;

			case layer_3:
				conv_kernel_size = (uint8_t)head->con_layer_kernel_3;
				pool_kernel_size = (uint8_t)head->con_layer_pooling_3;
				break;
		}

		if (i != layer)//不是最后一层卷积 - Not the last convolution
		{
			output_image_size = (uint8_t)floor((output_image_size + 2 * 0 - 1 * (conv_kernel_size - 1) - 1) / (float)1 + 1);
			output_image_size = (uint8_t)floor((output_image_size + 2 * 0 - 1 * (pool_kernel_size - 1) - 1) / (float)pool_kernel_size + 1);
		}
		else
		{	
			if (conv_pool == conv) //如果是卷积 - If it is convolution
			{
				output_image_size = (uint8_t)floor((output_image_size + 2 * 0 - 1 * (conv_kernel_size - 1) - 1) / (float)1 + 1);
			}
			else//如果是池化 - If it is pooling
			{
				output_image_size = (uint8_t)floor((output_image_size + 2 * 0 - 1 * (conv_kernel_size - 1) - 1) / (float)1 + 1);
				output_image_size = (uint8_t)floor((output_image_size + 2 * 0 - 1 * (pool_kernel_size - 1) - 1) / (float)pool_kernel_size + 1);
			}
		}
	}

	return output_image_size;
}

// conv image input size 
uint16_t lw_conv_pool_input_size(lw_head_t* head, lw_layer_t layer, lw_conv_pool_t conv_pool)
{
	uint8_t conv_kernel_size = 0;
	uint8_t pool_kernel_size = 0;

	uint16_t output_image_size = 0; //每层的输入图像大小 - Input image size for each layer

	//如果是卷积 - If it is convolution
	if (conv_pool == conv) 
	{
		uint16_t image_size = 0; //每层的输入图像大小 - Input image size for each layer

		if (layer == layer_1) //如果是第一层 - If it is the first layer
		{
			image_size = (uint16_t)head->image_size;
		}
		else//如果不是第一层 - If it is not the first layer
		{
			image_size = (uint16_t)head->image_size;

			for (lw_layer_t i = (lw_layer_t)1; i < (lw_layer_t)(layer + 1 - 1); i++)
			{
				switch (i)
				{
					case layer_1:
						conv_kernel_size = (uint8_t)head->con_layer_kernel_1;
						pool_kernel_size = (uint8_t)head->con_layer_pooling_1;
						break;

					case layer_2:
						conv_kernel_size = (uint8_t)head->con_layer_kernel_2;
						pool_kernel_size = (uint8_t)head->con_layer_pooling_2;
						break;

					case layer_3:
						conv_kernel_size = (uint8_t)head->con_layer_kernel_3;
						pool_kernel_size = (uint8_t)head->con_layer_pooling_3;
						break;
				}

				image_size = (uint16_t)floor((image_size + 2 * 0 - 1 * (conv_kernel_size - 1) - 1) / (float)1 + 1); //卷积输出 - Convolution output
				image_size = (uint16_t)floor((image_size + 2 * 0 - 1 * (pool_kernel_size - 1) - 1) / (float)pool_kernel_size + 1); //池化输出 - Pooling output
			}
		}

		output_image_size = image_size;
	}
	else //如果是pool - If it is pool
	{
		uint16_t image_size = (uint16_t)head->image_size;; //每层的输入图像大小 - Input image size for each layer

		for (lw_layer_t i = (lw_layer_t)1; i < (lw_layer_t)(layer + 1); i++)
		{
			switch (i)
			{
				case layer_1:
					conv_kernel_size = (uint8_t)head->con_layer_kernel_1;
					pool_kernel_size = (uint8_t)head->con_layer_pooling_1;
					break;

				case layer_2:
					conv_kernel_size = (uint8_t)head->con_layer_kernel_2;
					pool_kernel_size = (uint8_t)head->con_layer_pooling_2;
					break;

				case layer_3:
					conv_kernel_size = (uint8_t)head->con_layer_kernel_3;
					pool_kernel_size = (uint8_t)head->con_layer_pooling_3;
					break;
			}

			if (i == layer)//是最后一层 - Is the last layer
			{
				image_size = (uint8_t)floor((image_size + 2 * 0 - 1 * (conv_kernel_size - 1) - 1) / (float)1 + 1); //卷积
			}
			else
			{
				image_size = (uint8_t)floor((image_size + 2 * 0 - 1 * (conv_kernel_size - 1) - 1) / (float)1 + 1);
				image_size = (uint8_t)floor((image_size + 2 * 0 - 1 * (pool_kernel_size - 1) - 1) / (float)pool_kernel_size + 1); //池化
			}
		}

		output_image_size = image_size;
	}

	return output_image_size;
}

//读取flash - Read flash
void lw_read_flash(float* read_buffer, uint32_t read_len, uint32_t seek)
{
	uint32_t flash_addr = (uint32_t)FLASH_DATA_BASE_ADDR + seek;
	for (uint32_t i = 0; i < read_len; i++)
	{
		read_buffer[i] = *(float*)(flash_addr);
		flash_addr += 4; //地址偏移4个字节 - Address offset 4 bytes
	}
}

//读取SD - Read SD
void lw_read_sd(float* read_buffer, uint32_t read_len, uint32_t seek)
{
	
}

