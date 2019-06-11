#include "stm32f4xx_hal.h"
#include "sounds.h"
#include <math.h>
#include <stdlib.h>

int8_t currentOctave = 4;
float audioFreqRoot = powf(2.0f, 1.0f/12.0f);
int8_t currentOSC = 0;

// get OSC type
OSC_TYPE getOSCType(uint8_t i) {
	switch(i) {
	case 0:
		return SIN;
	case 1:
		return SQUARE;
	case 2:
		return TRIANGLE;
	case 3:
		return SAW_AN;
	case 4:
		return NOISE;
	case 5:
		return PIANO;
	default:
		return SIN;
	}
}

// used to generate correct output of different waves
float omega(float freq, float Pi) {
	return Pi * PI * freq/F_BASE;
}

// used to generate sound of different waves
float generateSinWave(float freq, uint16_t time) {
	float w = omega(freq, 2.0f);
	float val = sinf(w * time);
	return val;
}

float generateSquareWave(float freq, uint16_t time) {
	float w = omega(freq, 2.0f);
	float val = sinf(w * time);
	val = (val >= 0)? 1: -1;
	return val;
}

float generateTriangleWave(float freq, uint16_t time) {
	float w = omega(freq, 2.0f);
	float val = asinf(sinf(w * time)) * (2.0f/PI);
	return val;

}

float generateSawAnalogWave(float freq, uint16_t time) {
	float w = omega(freq, 2.0f);
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

float generatePiano(float freq, uint16_t time) {
	float val = sinf(omega(freq, 2.0f) * time) * 0.5f + (0.0f - cosf(omega(freq, 2.0f) * time));
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
	case PIANO:
		return generatePiano(freq, time);
	default:
		return 0;
	}
}

float synthesizeSound(float freq, uint16_t time, OSC_TYPE current_osc, NoteEnvelope envelope) {
	float sound = osc(freq, time, current_osc);

	if(envelope.is_playing) {
		if(envelope.time <= envelope.attackTime) {
			sound *= ((envelope.time / envelope.attackTime) * envelope.startAmplitude);
		}

		if (envelope.time > envelope.attackTime && envelope.time <= envelope.attackTime + envelope.decayTime) {
			sound *= (envelope.startAmplitude - (envelope.startAmplitude - (envelope.sustainAmplitude * ((envelope.time - envelope.attackTime) / envelope.decayTime))));
		}

		if (envelope.time > envelope.attackTime + envelope.decayTime) {
			sound *= envelope.sustainAmplitude;
		}
	}

	if(envelope.is_releasing) {
		sound *= (0 - (envelope.sustainAmplitude * (envelope.time / envelope.releaseTime)) + envelope.sustainAmplitude);
	}

	return sound;
}

/* We generate the array of sounds */
float generateWaves(Note s, unsigned long time, OSC_TYPE current_osc, NoteEnvelope envelope) {
	float freq;

	uint16_t modTime;

	switch(s) {
	case TEST:
		freq = 1.0f;
		modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case C:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 3);
        modTime = time%(unsigned long)(F_BASE/freq);
        return synthesizeSound(freq, modTime, current_osc, envelope);
	case Cs:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 4);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case D:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 5);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case Ds:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 6);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case E:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 7);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case F:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 8);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case Fs:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 9);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case G:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 10);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case Gs:
		freq = (BASE_A_FREQ * powf(2, currentOctave)) * powf(audioFreqRoot, 11);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case A:
		freq = (BASE_A_FREQ * powf(2, currentOctave + 1)) * powf(audioFreqRoot, 0);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case Bf:
		freq = (BASE_A_FREQ * powf(2, currentOctave + 1)) * powf(audioFreqRoot, 1);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	case B:
		freq = (BASE_A_FREQ * powf(2, currentOctave + 1)) * powf(audioFreqRoot, 2);
        modTime = time%(unsigned long)(F_BASE/freq);
		return synthesizeSound(freq, modTime, current_osc, envelope);
	default:
		return 0.0f;
	}
}
