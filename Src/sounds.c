#include "stm32f4xx_hal.h"
#include "sounds.h"
#include <math.h>

uint8_t currentOctave = 1;
int16_t waveData[I2S_DATA_SIZE];
float audioFreqRoot = powf(2.0f, 1.0f/12.0f);

// used to generate correct output of different waves
float omega(freq) {
	return 2 * PI * freq/F_BASE;
}

// used to generate sound of different waves
void generateSinWave(freq) {
	float w = omega(freq);
	float val;
	for(uint16_t i = 0; i < F_BASE/freq; i++) {
		val = sinf(i * w);
		waveData[i*2] = (val) * 8000;
		waveData[i * 2 + 1] = (val) * 8000;
	}
}

void generateSquareWave(freq) {
	float w = omega(freq);
	float val;
	for(uint16_t i = 0; i < F_BASE/freq; i++) {
		val = sinf(i * w);
		waveData[i*2] = ((val > 0)? 1:-1) * 8000;
		waveData[i * 2 + 1] = ((val > 0)? 1:-1) * 8000;
	}
}

void generateTriangleWave(freq) {
	float w = omega(freq);
	float val;
	for(uint16_t i = 0; i < F_BASE/freq; i++) {
		val = asinf(sinf(i * w));
		waveData[i*2] = ((val > 0)? 1:-1) * 8000;
		waveData[i * 2 + 1] = ((val > 0)? 1:-1) * 8000;
	}
}

void generateSawAnalogueWave(freq) {
	float w = omega(freq);
	float val = 0;
	for(uint16_t i = 0; i < F_BASE/freq; i++) {
		val += (sinf(i * w)) / (float)i;
		waveData[i*2] = ((val > 0)? 1:-1) * 8000;
		waveData[i * 2 + 1] = ((val > 0)? 1:-1) * 8000;
	}
}

void osc(float freq, OSC_TYPE type) {
	switch(type) {
	case SIN:
		return generateSinWave(freq);
	case SQUARE:
		return generateSquareWave(freq);
	case TRIANGLE:
		return generateTriangleWave(freq);
	case SAW_AN:
		return generateSawAnalogueWave(freq);
	default:
		return;
	}
}

void synthesizeSound(freq) {
	osc(freq, SIN);
}

/* We generate the array of sounds */
void generateWaves(Note s) {
	float freq;

	switch(s) {
	case C:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 3);
		return synthesizeSound(freq);
	case Cs:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 4);
		return synthesizeSound(freq);
	case D:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 5);
		return synthesizeSound(freq);
	case Ds:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 6);
		return synthesizeSound(freq);
	case E:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 7);
		return synthesizeSound(freq);
	case F:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 8);
		return synthesizeSound(freq);
		break;
	case Fs:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 9);
		return synthesizeSound(freq);
	case G:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 10);
		return synthesizeSound(freq);
	case Gs:
		freq = BASE_A_FREQ * powf(audioFreqRoot, 11);
		return synthesizeSound(freq);
	case A:
		freq = (BASE_A_FREQ * (currentOctave + 1)) * powf(audioFreqRoot, 0);
		return synthesizeSound(freq);
	case Bf:
		freq = (BASE_A_FREQ * (currentOctave + 1)) * powf(audioFreqRoot, 1);
		return synthesizeSound(freq);
	case B:
		freq = (BASE_A_FREQ * (currentOctave + 1)) * pow(audioFreqRoot, 2);
		return synthesizeSound(freq);
	default:
		return;
	}
}

void mute(I2S_HandleTypeDef hi2s3) {
	HAL_I2S_DMAStop(&hi2s3);

	for(int i = 0; i < I2S_DATA_SIZE; i++) {
		waveData[i] = 0;
	}

	HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)waveData, 2);
}
