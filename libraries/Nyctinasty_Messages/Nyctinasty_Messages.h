#ifndef Nyctinasty_Messages_h
#define Nyctinasty_Messages_h

#include <Arduino.h>

// system states. probably want to implement a FSM on each uC to work with these.
enum systemState {
//	state		//	description
	STARTUP=0,	//	all roles start here
	
	LONELY,		// nobody is messing with the project; all quiet, or maybe an attractant.
	OHAI,		// human walked through the sensors; 'sup, earthling!
	GOODNUF,	// one human is doing something rhythmic; or two humans are messing around
	GOODJOB,	// two humans are dosing something rhythmic; or three humans are messing around
	WINNING, 	// three somebodies are crushing it

	REBOOT,		//  trigger to reboot  

	N_STATES	//	as a counter/max
};

// command structure
typedef struct {
	systemState state={STARTUP}; 
} SystemCommand;
	
// number of Arches per Sepal
#define N_ARCH 3

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
#define N_FREQ_SAMPLES (uint16_t)(1<<7)  // This value MUST ALWAYS be a power of 2

// time required for buffer fill
#define FILL_TIME DISTANCE_SAMPLING_RATE*N_FREQ_SAMPLES // 1280 ms

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

// water works control
enum pumpState {
	// pretty self-explanatory.  these are useful when a HIGH pin state means OFF.
	OFF,		// 0==LOW
	ON			// 1==HIGH
};
enum routeState {
	FOUNTAIN,	// 0==LOW
	CANNON		// 1==HIGH
};
typedef struct {
	// state of the four pumps; with default
	// pump pair with priming, so one of these must be on first
	pumpState primePump[2] = {OFF, OFF};
	// pump pair without priming
	pumpState boostPump[2] = {OFF, OFF};
	// state of the output; with default
	routeState route = {FOUNTAIN};
} WaterWorks;

#endif