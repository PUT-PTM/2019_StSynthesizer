/* Header file of a sound class, used to generate sounds */
/* Our project is running at 50kHz */

// sound note
typedef enum {C, Cs, D, Ds, E, F, Fs , G, Gs, A, Bf, B} Note;

// oscilator type
typedef enum { SIN, SQUARE, TRIANGLE, SAW_AN, SAW_OP } OSC_TYPE;

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
void generateWaves(Note s);
void mute();

// Base frequency
#define F_BASE 48000.0f
// PI defintion
#define PI 3.14159f
// Notes in I2C array
#define I2S_DATA_SIZE 1600
// Lowest A frequency
#define BASE_A_FREQ 27.5f
// Count octave from 1 (not 0)
extern uint8_t currentOctave;
// The current data for our wave
extern int16_t waveData[I2S_DATA_SIZE];
