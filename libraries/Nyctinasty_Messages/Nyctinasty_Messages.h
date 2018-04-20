#ifndef Nyctinasty_Messages_h
#define Nyctinasty_Messages_h

#include <Arduino.h>

// system states. probably want to implement a FSM on each uC to work with these.
/*
* IDLE: Could want to idle stuff, I guess
* NORMAL: Arches operate sensors and lighting
* CENTRAL: Coordinator(s) operate lighting
* REBOOT: triggers each microcontroller to reboot.  
* REPROGRAM: triggers each microcontroller to contact the webserver to pull a new binary via OTA programming.
*/
enum systemState {
// note: starting at '48' implies that a plain text "0" will be interpreted as the zeroith element.
//state				default	subscriptions	publications	
  IDLE=48,		//	yes		ANY				NONE
  NORMAL,		//	no		ANY				ANY
  CENTRAL,		//	no		ANY				Coordinator
  
  REBOOT,		//  no		N/A				N/A
  REPROGRAM,	//  no		N/A				N/A

  N_STATES	// as a counter/max
};

// command structure
typedef struct {
	systemState state={NORMAL}; 
} SystemCommand;
	
// number of Sepals
#define N_SEPAL 3

// number of Arches per Sepal
#define N_ARCH 3

// number of distance Sensors per Arch
#define N_SENSOR 8

// distance data
typedef struct {
	// min and max range information
	uint16_t min, max; // ADC limits
	// noise level in readings; values less than this are sketchy
	uint16_t noise; // lower bits are noisy
	// proximity.  bigger numbers = closer to arch
	uint16_t prox[N_SENSOR];
} SepalArchDistance;

// FFT analysis of the distance data
#define N_FREQ_BINS 16 // number of discrete frequency bins to report
const uint8_t WeightFrequency[] = {
		0, // almost certainly don't want to use 0th bin
		1,1,1,1,1,1,1,1, // equal weight to next 8 bins
		0,0,0,0,0,0,0 // no weight on the high frequency bins
};
static_assert(
	sizeof(WeightFrequency) == N_FREQ_BINS*sizeof(WeightFrequency[0]),
	"Weight size incorrect."
);
typedef struct {
	// average power; something like the mean of the frequency bins
	uint16_t avgPower[N_SENSOR]={0};
	// frequency power
	uint16_t power[N_SENSOR][N_FREQ_BINS]={{0}};
	// the specific frequency of power[i][j] is (j+1)*distanceSampleRate/N_FREQ_SAMPLES
} SepalArchFrequency;

#endif