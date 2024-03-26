#ifndef __STFT__
#define __STFT__

	float* lw_stft_fft(lw_head_t* head, float* stft_input); // stft,调用DSP库函数FFT - stft, call dsp lib function FFT
	float* lw_stft_dft(lw_head_t* head, float* stft_input); // stft,纯手写DFT,FFT是DFT的快速算法 - stft, pure hand-written DFT, FFT is a fast algorithm for DFT


#endif


