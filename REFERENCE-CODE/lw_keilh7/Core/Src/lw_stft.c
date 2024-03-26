#include "main.h"


//调用DSP库函数FFT - Call DSP Library Function FFT
float* lw_stft_fft(lw_head_t* head, float* stft_input)
{
		float pi = 3.1415926f;
	  uint32_t stft_input_len = (head->image_size - 1) * ((head->image_size *2)/2) + (head->image_size *2); //做一张图片STFT的信号长度,计算公式参考mycnn.py 256行左右 - The length of the signal for a picture STFT, the calculation formula refers to mycnn.py around line 256
		uint32_t window_len = head->image_size * 2; //做FFT的长度 - The length of FFT
	  uint32_t shift_len = window_len/2; //移动步长 - Move step length
	
	  float* stft_output = (float*)calloc(head->image_size *sizeof(float) *head->image_size *sizeof(float), 1);  //给stft输出申请内存,只取单边频谱 - Allocate memory for stft output, only take one-sided spectrum

		// DSP FFT库 - DSP FFT Library
		//arm_cfft_radix4_instance_f32 scfft; //基4 - base 4
		//arm_cfft_radix4_init_f32(&scfft, window_len, 0, 1); //初始化结构体,FFT点数-类型(0:FFT, 1:IFFT)-按位取反 - Initialize the structure, FFT number-type (0: FFT, 1: IFFT)-bitwise negation
		arm_cfft_radix2_instance_f32 scfft; //基2
	  arm_cfft_radix2_init_f32(&scfft, window_len, 0, 1); //初始化结构体,FFT点数-类型(0:FFT, 1:IFFT)-按位取反 - Initialize the structure, FFT number-type (0: FFT, 1: IFFT)-bitwise negation

	   // 汉宁窗函数 - Hanning window function
    float *hamming_window = (float*)calloc(window_len * sizeof(float), 1); // 汉宁窗 - Hanning window
    for (uint32_t i = 0; i < window_len; i++)
        hamming_window[i] = (float)(0.5 -0.5 * cos(2 * pi * i / (window_len - 1)));

    // 遍历信号对每一帧做FFT - Traverse the signal to do FFT for each frame
    for (uint32_t i = 0; i < (stft_input_len - window_len)/(shift_len) + 1; i++) 
    {
        float* window_data = (float*)calloc(window_len * sizeof(float), 1); // 当前窗口 - Current window
        memmove(window_data, stft_input + i * shift_len, sizeof(float) * window_len); //生成窗口数据 - Generate window data

         //当前数据乘以汉宁窗 - The current data is multiplied by the Hanning window
				float* hamming_data = (float*)calloc(2 * window_len * sizeof(float), 1); //加窗数据，实部和虚部在数组中交叉，虚部全为0 - Windowed data, real and imaginary parts are interleaved in the array, and the imaginary part is all 0
        for (uint32_t j = 0; j < window_len; j++) 
        {
					hamming_data[2*j] = window_data[j] * hamming_window[j]; //加窗覆盖掉原来的数据，实部置为实际数据 - The window covers the original data, and the real part is set to the actual data
        }

        // FFT
				//arm_cfft_radix4_f32(&scfft, hamming_data);	//基4, 输入长度只能为4^n(16/64/256/1024/4096), 计算FFT,输出的数据通过覆盖的方式也放在hamming_data中,包含实部与虚部 - Base 4, the input length can only be 4^n (16/64/256/1024/4096), calculate the FFT, and the output data is also placed in hamming_data by overwriting, including the real and imaginary parts
				arm_cfft_radix2_f32(&scfft, hamming_data);	//基2, 输入长度只能为2^n(16/64/128/256/512/1024/2048/4096), 计算FFT,输出的数据通过覆盖的方式也放在hamming_data中,包含实部与虚部,基4快于基2 - Base 2, the input length can only be 2^n (16/64/128/256/512/1024/2048/4096), calculate the FFT, and the output data is also placed in hamming_data by overwriting, including the real and imaginary parts, base 4 is faster than base 2

				// fft变换后的数据 - Data after FFT transformation
				float *fft_out = (float*)calloc(window_len * sizeof(float), 1); // 保存求模后的数据,输出为双边频谱 - Save the data after the modulus is taken, and the output is a two-sided spectrum
   			arm_cmplx_mag_f32(hamming_data, fft_out, window_len);	//求模 - modulus
				
				// 归一化 - Normalization
				for (uint32_t k = 0; k < window_len; k++) 
				{
					fft_out[k] = fft_out[k]/(window_len/2.0);
				}

        memmove(stft_output + i * window_len/2, fft_out, sizeof(float) * window_len/2); //可以只取一半的频谱, window_len/2 - Only half of the spectrum can be taken, window_len/2
        free(window_data);
        free(hamming_data);
        free(fft_out);
    }

    free(hamming_window);
		
		return stft_output;
}

//纯手写DFT,FFT是DFT的快速算法 - Pure handwritten DFT, FFT is a fast algorithm for DFT
float* lw_stft_dft(lw_head_t* head, float* stft_input)
{
		float pi = 3.1415926f;
	  uint32_t stft_input_len = (head->image_size - 1) * ((head->image_size *2)/2) + (head->image_size *2); //做一张图片STFT的信号长度,计算公式参考mycnn.py 256行左右 - The length of the signal for a picture STFT, the calculation formula refers to mycnn.py around line 256
		uint32_t window_len = head->image_size *2; //做FFT的长度 - The length of FFT
	  uint32_t shift_len = window_len/2; //移动步长 - Move step length
	
	  float* stft_output = (float*)calloc(head->image_size *sizeof(float) *head->image_size *sizeof(float), 1);  //给stft输出申请内存,只取单边频谱 - Allocate memory for stft output, only take one-sided spectrum
	
	  // 汉宁窗函数 - Hanning window function
    float *hamming_window = (float*)calloc(window_len * sizeof(float), 1); // 汉宁窗 - Hanning window
    for (uint32_t i = 0; i < window_len; i++)
        hamming_window[i] = (float)(0.5 -0.5 * cos(2 * pi * i / (window_len - 1)));

	
    // 遍历信号对每一帧做FFT - Traverse the signal to do FFT for each frame
    for (uint32_t i = 0; i < (stft_input_len - window_len)/(shift_len) + 1; i++) 
    {
        float* window_data = (float*)calloc(window_len * sizeof(float), 1); // 当前窗口 - Current window
        memmove(window_data, stft_input + i * shift_len, sizeof(float) * window_len); //生成窗口数据 - Generate window data

         //当前数据乘以汉宁窗 - The current data is multiplied by the Hanning window
				float* hamming_data = (float*)calloc(window_len * sizeof(float), 1); //加窗数据 - Windowed data
        for (uint32_t j = 0; j < window_len; j++) 
        {
					hamming_data[j] = window_data[j] * hamming_window[j]; //加窗覆盖掉原来的数据 - The window covers the original data
        }

				// 做DFT,对称频率可以只计算一半window_len/2 - Do DFT, the symmetric frequency can only calculate half window_len/2
				float* dft_out = (float*)calloc(window_len * sizeof(float), 1);
				for (uint32_t k = 0; k < window_len; k++)
				{
					float real = 0; //实部 - Real part
					float imag = 0; //虚部 - Imaginary part
					for (uint32_t n = 0; n < window_len; n++)
					{
							real += (hamming_data[n] * cos(2 * pi * k * n / window_len));
							imag += (hamming_data[n] * sin(2 * pi * k * n / window_len));
					}
					dft_out[k] = sqrt(real * real + imag * imag)/(window_len/2); //取模并归一化(window_len/2.0) - Take modulus and normalize (window_len/2.0)
				}
				
				memmove(stft_output + i * window_len/2, dft_out, sizeof(float) * window_len/2); //可以只取一半的频谱, window_len/2 - Only half of the spectrum can be taken, window_len/2 

        free(window_data);
        free(hamming_data);
        free(dft_out);
    }

    free(hamming_window);
		
		return stft_output;
}
