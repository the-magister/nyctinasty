#ifndef Nyctinasty_Messages_h
#define Nyctinasty_Messages_h

#include <Arduino.h>
#include <FastLED.h> // for CRGB definition

// command structure
typedef struct {
	// send fps, frame = SensorReading
	uint8_t fps=100;
} SystemCommand;

// number of seeples
#define N_SEEPLES 3

// number of arches per seeple
#define N_ARCHES 3

// distance sensors on the seeple arches
#define N_SENSOR 8
typedef struct {
	// min and max range information
	uint16_t min=0, max=(1<<12)-1; // 12-bit ADC
	
	// noise level in readings; values less than this are sketchy
	uint16_t noise=(1<<5)-1; // lower bits are noisy
	
	// proximity.  bigger numbers = closer to arch
	uint16_t prox[N_SENSOR]={0};
} SeepleArchDistance;
// size_of = 2*(3+8) = 22 bytes

// FFT analysis of the distance data
#define N_FREQ_SAMPLES (uint16_t)(1<<8)  // This value MUST ALWAYS be a power of 2
#define N_FREQ_BINS ((N_FREQ_SAMPLES)/2-1) // number of discrete frequency bins 
typedef struct {
	// average power; something like the mean of the frequency bins
	double avgPower[N_SENSOR]={0.0};
	// frequency power
	double power[N_SENSOR][N_FREQ_BINS]={{0.0}};
	// sampling rate of each sensor, Hz
	double samplingFrequency={0};
	// the specific frequency of power[i][j] is (j+1)*samplingFrequency/N_FREQ_SAMPLES
} SeepleArchFreq;

// lights on the seeple arches, as a crazy-big structure
#define N_LEDS_UP 16
#define N_LEDS_DOWN 16
typedef struct {
	CRGB bar[N_SENSOR]; // seepleArchBar[0] leftmost
	CRGB leftUp[N_LEDS_UP]; // leftUp[NUM_LEDS_UP-1] leftmost and top of the up segment
	CRGB rightUp[N_LEDS_UP]; // rightUp(5,6) rightmost; up segment's 5th and 6th led.
	CRGB leftDown[N_LEDS_DOWN]; // leftDown.fill_rainbow(HUE_BLUE, 5) left "leg" is colorful.
	CRGB rightDown[N_LEDS_DOWN]; // rightDown[1].blur1d(128) rightmost top of the down segment is blurry
} SeepleArchLight;
// size_of = 3*(8+2*16+2*16) = 216 bytes

#endif