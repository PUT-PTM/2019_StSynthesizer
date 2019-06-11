/* Header file of a sound class, used to generate sounds */
/* Our project is running at 50kHz */

// sound note
typedef enum {TEST, C, Cs, D, Ds, E, F, Fs , G, Gs, A, Bf, B} Note;

// oscilator type
typedef enum { SIN, SQUARE, TRIANGLE, SAW_AN, NOISE, PIANO } OSC_TYPE;

typedef struct {
	uint8_t is_playing;
	uint8_t is_releasing;

	float attackTime;
	float decayTime;
	float releaseTime;

	float sustainAmplitude;
	float startAmplitude;

	float time;
} NoteEnvelope;

// Defining a struct sound
typedef struct {
	// The note we're playing
	Note note;
	// Current time on the wave
	uint16_t time;
	// How long does this last for after we release the key
	uint16_t duration;
	// How long to come to full sound after we hit the note
	uint16_t attack;
	// Is playing
	uint8_t playing;
} Sound;

/* Function used to generate sound waves based on the given note */
float generateWaves(Note s, unsigned long tim, OSC_TYPE osc, NoteEnvelope envelope);
OSC_TYPE getOSCType(uint8_t i);

// Base frequency
#define F_BASE 16000.0f
// PI defintion
#define PI 3.14159f
// Lowest A frequency
#define BASE_A_FREQ 27.5f
// Count octave from 1 (not 0)
extern int8_t currentOctave;
extern int8_t currentOSC;

extern unsigned long time;
