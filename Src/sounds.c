#include "stm32f4xx_hal.h"
#include "sounds.h"
#include <math.h>
#include <stdlib.h>

uint8_t currentOctave = 4;
float audioFreqRoot = powf(2.0f, 1.0f/12.0f);

// used to generate correct output of different waves
float omega(float freq) {
	return 2.0f * PI * freq/F_BASE;
}

// used to generate sound of different waves
float generateSinWave(float freq, uint16_t time) {
	float w = omega(freq);
	float val = sinf(w * time);
	return val;
}

float generateSquareWave(float freq, uint16_t time) {
	float w = omega(freq);
	float val = sinf(w * time);
	val = (val >= 0)? 1: -1;
	return val;
}

float generateTriangleWave(float freq, uint16_t time) {
	float w = omega(freq);
	float val = asinf(sinf(w * time)) * (2.0f/PI);
	return val;

}

float generateSawAnalogWave(float freq, uint16_t time) {
	float w = omega(freq);
	float val = 0.0f;

	for (float n = 1.0f; n < 30.0f; n++) {
		val += (sinf(n * w * time)) / n;
	}

	return -(val * (2.0f / PI));
}

float generateNoise() {
	float val = 2.0f * ((float) rand() / (float) RAND_MAX) - 1.0f;
	return val;
}

float osc(float freq, uint16_t time, OSC_TYPE type) {
	switch(type) {
	case SIN:
		return generateSinWave(freq, time);
	case SQUARE:
		return generateSquareWave(freq, time);
	case TRIANGLE:
		return generateTriangleWave(freq, time);
	case SAW_AN:
		return generateSawAnalogWave(freq, time);
	case NOISE:
		return generateNoise();
	default:
		return 0;
	}
}

float synthesizeSound(float freq, uint16_t time) {
	return osc(freq, time, SAW_AN);
}

/* We generate the array of sounds */
float generateWaves(Note s, unsigned long time) {
	float freq;

	uint16_t modTime;

	switch(s) {
	case TEST:
		freq = 1.0f;
		modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case C:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 3);
        modTime = time%(unsigned long)(F_BASE/freq);
        return synthesizeSound(freq, modTime);
	case Cs:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 4);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case D:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 5);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case Ds:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 6);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case E:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 7);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case F:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 8);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case Fs:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 9);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case G:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 10);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case Gs:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 11);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case A:
		freq = (BASE_A_FREQ * powf(2, currentOctave + 1)) * powf(audioFreqRoot, 0);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case Bf:
		freq = (BASE_A_FREQ * powf(2, currentOctave + 1)) * powf(audioFreqRoot, 1);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	case B:
		freq = (BASE_A_FREQ * powf(2, currentOctave + 1)) * powf(audioFreqRoot, 2);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime);
	default:
		return 0.0f;
	}
}
