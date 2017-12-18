#ifndef Nyctinasty_Messages_h
#define Nyctinasty_Messages_h

#include <Arduino.h>
#include <FastLED.h> // for CRGB definition

// system states. probably want to implement a FSM on each uC to work with these.
/*
* STARTUP: allows subscriptions to get processed and no publications are allowed to clear comms channels.  after a delay, each Coordinator sends DISTRIB.
* NORMAL: Coordinator/S are controlling each arch.  
* CENTRAL: Coordinator is controlling each arch.
* REBOOT: triggers each microcontroller to reboot.  
* REPROGRAM: triggers each microcontroller to contact the webserver to pull a new binary via OTA programming.
*/
enum systemState {
//state				default	subscriptions	publications	
  STARTUP=0,	//	yes		ANY				NONE
  NORMAL,		//	no		ANY				ANY
  CENTRAL,		//	no		ANY				Coordinator
  
  REBOOT,		//  no		N/A				N/A
  REPROGRAM,	//  no		N/A				N/A

  N_STATES	// as a counter/max
};

// command structure
typedef struct {
	systemState state={STARTUP}; // critical that this is defaulted to STARTUP
} SystemCommand;

// number of Sepals
#define N_SEPALS 3

// number of arches per Sepal
#define N_ARCHES 3

// distance sensors on the Sepal arches
#define N_SENSOR 8
const uint32_t distanceSampleRate = 10; // ms, 50 Hz
typedef struct {
	// track provenance
	byte sepal, arch;
	// min and max range information
	uint16_t min, max; // ADC limits
	// noise level in readings; values less than this are sketchy
	uint16_t noise; // lower bits are noisy
	// proximity.  bigger numbers = closer to arch
	uint16_t prox[N_SENSOR];
} SepalArchDistance;
// size_of = 2*(3+8)+2 = 24 bytes

// FFT analysis of the distance data
#define N_FREQ_BINS 16 // number of discrete frequency bins to report
typedef struct {
	// track provenance
	byte sepal, arch;
	// average power; something like the mean of the frequency bins
	uint16_t avgPower[N_SENSOR]={0};
	// frequency power
	uint16_t power[N_SENSOR][N_FREQ_BINS]={{0.0}};
	// the specific frequency of power[i][j] is (j+1)*distanceSampleRate/N_FREQ_SAMPLES
} SepalArchFreq;

// lights on the Sepal arches, as a crazy-big structure
#define N_LEDS 16
typedef struct {
	byte sepal, arch; // track provenance
	CRGB bar[N_SENSOR]; // SepalArchBar[0] leftmost
	CRGB leftUp[N_LEDS]; // leftUp[NUM_LEDS_UP-1] leftmost and top of the up segment
	CRGB rightUp[N_LEDS]; // rightUp(5,6) rightmost; up segment's 5th and 6th led.
	CRGB leftDown[N_LEDS]; // leftDown.fill_rainbow(HUE_BLUE, 5) left "leg" is colorful.
	CRGB rightDown[N_LEDS]; // rightDown[1].blur1d(128) rightmost top of the down segment is blurry
} SepalArchLight;
// size_of = 3*(8+2*16+2*16) = 216 bytes

#endif