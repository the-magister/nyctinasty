#ifndef Nyctinasty_Messages_h
#define Nyctinasty_Messages_h

#include <Arduino.h>
	
// number of Arches per Sepal
#define N_ARCH 3

// system states. probably want to implement a FSM on each uC to work with these.
enum systemState {
//	state		//	description
	STARTUP=0,	//	all roles start here
	
	LONELY,		// 0 players
	OHAI,		// 1 players
	GOODNUF,	// 2 players
	GOODJOB,	// 3 players or 2 players coordinated
	WINNING, 	// 3 players and 2 players coordinated
	FANFARE,	// 3 players and 3 players coordinated 
	
	REBOOT,		//  trigger to reboot 
	N_STATES	//	as a counter/max
};

// command structure
typedef struct {
	systemState state={STARTUP}; 
	boolean isPlayer[N_ARCH] = {false}; // A0, A1, A2
	boolean areCoordinated[N_ARCH] = {false}; // A0:A1, A1:A2, A2:A0
} SystemCommand;

// number of distance Sensors per Arch
#define N_SENSOR 6

// distance data
#define DISTANCE_SAMPLING_RATE 10UL // ms
#define DISTANCE_SAMPLING_FREQ (1000UL/DISTANCE_SAMPLING_RATE) // 100 Hz
typedef struct {
	// min and max range information
	uint16_t min, max; // ADC limits
	// noise level in readings; values less than this are sketchy
	uint16_t noise; // lower bits are noisy
	// proximity.  bigger numbers = closer to arch
	uint16_t prox[N_SENSOR];
} SepalArchDistance;

// FFT analysis of the distance data
#define N_FREQ_SAMPLES (uint16_t)(1<<8)  // This value MUST ALWAYS be a power of 2

// time required for buffer fill
#define FILL_TIME DISTANCE_SAMPLING_RATE*N_FREQ_SAMPLES 

// Nyquist limit; can't detect frequencies faster than this
#define NYQUIST_LIMIT DISTANCE_SAMPLING_FREQ/2 // 50 Hz

// but, given Nyquist limit, only the first half of the bins are interpretable
// still, we just don't expect people to move at 50 Hz, right?  
// take the first handful (low freq) bins as relating to meaningful human motion.
#define HIGHEST_FREQ_BIN 16 // max: (N_FREQ_SAMPLES/2)
#define N_FREQ_BINS HIGHEST_FREQ_BIN
typedef struct {
	// average power; something like the mean of the frequency bins
	uint16_t avgPower[N_SENSOR]={0};
	// frequency power
	uint16_t power[N_SENSOR][N_FREQ_BINS]={{0}};
		// the specific frequency of power[i][j] is (j+1)*DISTANCE_SAMPLING_FREQ/N_FREQ_SAMPLES in Hz
	// the (interpolated) frequency of maximum power per sensor
	float peakFreq[N_SENSOR]={0};
} SepalArchFrequency;

// cannon trigger
enum triggerState {
	TRIGGER_OFF,		// 0==LOW
	TRIGGER_ON			// 1==HIGH
};
typedef struct {
	triggerState state = TRIGGER_OFF;
} CannonTrigger;

#endif
