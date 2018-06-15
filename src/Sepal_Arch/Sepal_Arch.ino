// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"

#include <Streaming.h>
#include <Metro.h>
#include <arduinoFFT.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include <FiniteStateMachine.h>
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#include <FastLED.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// spammy?
#define SHOW_SERIAL_DEBUG true

// my role and arch number
NyctRole myRole = Arch2; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM
// Arch0, Arch1, Arch2 for bootstrapping a new controller.
byte myArch;

// wire it up
#define RX D1 // GPIO5
#define TX D2 // GPIO4 
// voltage divider: 5V->3.3V
// TX to RX_ESP via 820 Ohm resistor
// RX_ESP to GND via 1500 Ohm resistor
// LEDs are connected:
// D5, GPIO14
// D6, GPIO12
// D7, GPIO13
// D8, GPIO15
// also used
// D4, GPIO2, BUILTIN_LED

// LED handling

// our internal storage, mapped to the hardware.
// pay no attention to the man behind the curtain.
#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip
#define NUM_PINS 4
#define LEDS_BAR 3
#define LEDS_VERT 20
#define LEDS_PER_PIN LEDS_BAR+LEDS_VERT

CRGBArray<LEDS_PER_PIN> leftBack;
CRGBArray<LEDS_PER_PIN> rightBack;
CRGBArray<LEDS_PER_PIN> leftFront;
CRGBArray<LEDS_PER_PIN> rightFront;

// verticals
CRGBSet leftUp = leftFront(LEDS_BAR, LEDS_PER_PIN-1);
CRGBSet rightUp = rightFront(LEDS_BAR, LEDS_PER_PIN-1);
CRGBSet leftDown = leftBack(LEDS_BAR, LEDS_PER_PIN-1);
CRGBSet rightDown = rightBack(LEDS_BAR, LEDS_PER_PIN-1);

// color choices, based on arch and sepal information
byte archHue[N_ARCH] = {HUE_RED, HUE_GREEN, HUE_BLUE};
byte archSat[N_ARCH] = {128, 128, 128};

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);
void offline(); State Offline = State(offline);
void online(); State Online = State(online);
void slaved(); State Slaved = State(slaved);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
void reprogram() { comms.reprogram("Sepal_Arch.ino.bin"); }; State Reprogram = State(reprogram);
FSM stateMachine = FSM(Startup); // initialize state machine

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;

// our distance updates send as this structure as this topic
const uint32_t distancePublishRate = 1000UL/15UL; // ms
SepalArchDistance dist;

// our frequency updates send as this structure as this topic
// other frequency updates receive as this structure as this topic
typedef struct {
  boolean hasUpdate = false;
  SepalArchFrequency freq;
} sAF_t;
sAF_t sAF[N_ARCH];

// FFT object
uint16_t buffer[N_SENSOR][N_FREQ_SAMPLES];
boolean bufferReady = false;
arduinoFFT FFT = arduinoFFT();

// computed correspondence [-1,+1] between arches.
float concordTotal, concordNext, concordPrev;

// talk to the ADC device
SoftwareSerial mySerial(RX, TX, false, 1024);
SoftEasyTransfer ETin;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup begins.") << endl;

  // for remote output
  Serial << F("Configure softwareserial...");
  mySerial.begin(115200);
  // messages
  ETin.begin(details(dist), &mySerial);
  // after set up the input pin
  pinMode(TX, OUTPUT); // trigger to send
  Serial << F(" done.") << endl;

  // LEDs
  Serial << F("Configure leds...");
  FastLED.addLeds<WS2811, D5, COLOR_ORDER>(leftBack, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D6, COLOR_ORDER>(rightBack, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D7, COLOR_ORDER>(leftFront, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.addLeds<WS2811, D8, COLOR_ORDER>(rightFront, LEDS_PER_PIN).setCorrection(COLOR_CORRECTION);
  FastLED.setBrightness(255);
  Serial << F(" done.") << endl;

  // configure comms
  comms.begin(myRole);
  myRole = comms.getRole();
  myArch = myRole - 1;

  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  for ( byte i = 0; i < N_ARCH; i++ ) {
    // no need to subscribe to our own message
    if ( i != myArch ) comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, i);
  }

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

  // check for settings update
  if ( sC.hasUpdate ) {
    switchState(sC.settings.state);
    sC.hasUpdate = false;
  }

  // do stuff
  stateMachine.update();
}

void switchState(systemState state) {
  Serial << F("State.  Changing to ") << state << endl;
  switch ( state ) {
    case STARTUP: stateMachine.transitionTo(Startup); break;
    case OFFLINE: stateMachine.transitionTo(Offline); break;
    case ONLINE: stateMachine.transitionTo(Online); break;
    case SLAVED: stateMachine.transitionTo(Slaved); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void startup() {
  
  // spoof bar last
//  for( byte i=0; i<N_SENSOR; i++ ) dist.prox[i] = dist.max;

  static byte hue = archHue[myArch];
  EVERY_N_MILLISECONDS(10) {
    // show a throbbing rainbow background
    hue++;
    leftBack.fill_rainbow(hue, 255 / leftBack.size()); // paint
    rightBack = leftBack;
    leftFront.fill_rainbow(hue + 128, -255 / leftFront.size());
    rightFront = leftFront;

//    updateBarByDistance();

    pushToHardware();
  }

  // after 5 seconds, transition to Offline, but we could easily get directed to Online before that.
  static Metro startupTimeout(5000UL);
  if( startupTimeout.check() ) stateMachine.transitionTo(Offline);

}

void askForDistance() {
  // toggled TX pin
  static boolean pinState = false;
  static Metro distanceUpdate(DISTANCE_SAMPLING_RATE);

  if ( distanceUpdate.check() ) {
    distanceUpdate.reset();
    pinState = !pinState;
    digitalWrite(TX, pinState);
  }
}


/*
 * Order of operation is critical here.  We have three subsystems that use ISRs:
 *  FastLED: turns off ISRs
 *  WiFi: received information may be lost w/o ISR
 *  SoftwareSerial: received information may be lost w/o ISR
 *  
 *  From this, you can see that we need to be very careful with FastLED.show(), and 
 *  queue those up after we get SoftwareSerial data.  Can't really control when we get
 *  WiFi packets.
 *
 */
 
void offline() {
  if( comms.isConnected() ) {
    Serial << F("GOOD.  online!") << endl;
    stateMachine.transitionTo(Online);
  } else {
    normal(false);  
  }
}

void online() {
  if( comms.isConnected() ) {
    normal(true);
  } else {
    Serial << F("WARNING.  offline!") << endl;
    stateMachine.transitionTo(Offline);
  }
}

void normal(boolean isOnline) {
  static uint32_t counter = 0;
  static boolean publishReady = false;
  static byte fftIndex = 0;
  static Metro pushDistanceInterval(distancePublishRate);
  
  // check for an update to frequency data
  if ( sAF[0].hasUpdate || sAF[1].hasUpdate || sAF[2].hasUpdate ) {
    // update lower lights by frequency data
//      updateLegsByFrequency();
    updateLegsByPeakFreq();
    
    // compute concordance
//    getConcordance();

    // show it
//    updateTopsByConcordance();

    // reset
    sAF[0].hasUpdate = sAF[1].hasUpdate = sAF[2].hasUpdate = false;
  }

  // do FFT in segements when the buffer is ready
  if ( bufferReady ) {
    // compute
    computeFFT(fftIndex);

    // next sensor
    fftIndex++;

    // are we done?
    if ( fftIndex >= N_SENSOR ) {
      fftIndex = 0;
      bufferReady = false;
      publishReady = true;
    }
  }

  // check to see if we need to pull new distance data.
  askForDistance();

  // get data from ADCs
  if ( ETin.receiveData() ) {
    // just tracking actual distance update rate
    counter ++;

    // fill buffer
    fillBuffer();

    // update bar lights
    updateBarByDistance();

    // I don't actually know if sending via WiFi will disrupt SoftwareSerial; worth a look.
    if ( publishReady ) {
      publishReady = false;
      // publish
      if( isOnline ) comms.publish(&sAF[myArch].freq, myArch);
      // flag that our frequency data are ready
      sAF[myArch].hasUpdate = true;
    } else if( pushDistanceInterval.check() ) {
      // publish distance
      if( isOnline ) comms.publish(&dist, myArch);
      pushDistanceInterval.reset();
    }

    // we need to update the LEDs now while we won't screw up SoftwareSerial and WiFi
    pushToHardware();
  }

  const uint32_t reportInterval = 10;
  EVERY_N_SECONDS( reportInterval ) {
    uint32_t actualDISTANCE_SAMPLING_RATE = (reportInterval*1000UL)/counter;

    Serial << F("Distance sample interval, actual=") << actualDISTANCE_SAMPLING_RATE;
    Serial << F(" hypothetical=") << DISTANCE_SAMPLING_RATE;
    Serial << F(" ms.") << endl;
    
    counter=0;
  }
  
}
void slaved() {
  // NOP, currently.  Will look for lighting directions, either through UDP (direct) or MQTT (procedural).
}

// adjust lights on the bar with distance readings
void updateBarByDistance() {

  // compute using a dummy set of LEDs
  static CRGBArray<N_SENSOR> bar;
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    // quash noise
    if ( dist.prox[i] < dist.noise ) dist.prox[i] = dist.min;
    uint16_t intensity = map(
                           dist.prox[i],
                           dist.min, dist.max,
                           (uint16_t)0, (uint16_t)255
                         );
    bar[i] = CHSV(archHue[myArch], archSat[myArch], (byte)intensity);
  }

  // assign to hardware. ugly and direct, but we can see what's going on.
/*  
  leftBack[3] = leftFront[3] = bar[0];
  leftBack[2] = leftFront[2] = bar[1];
  leftBack[1] = leftFront[1] = bar[2];
  leftBack[0] = leftFront[0] = bar[3];

  rightBack[0] = rightFront[0] = bar[4];
  rightBack[1] = rightFront[1] = bar[5];
  rightBack[2] = rightFront[2] = bar[6];
  rightBack[3] = rightFront[3] = bar[7];
*/
  leftBack[2] = leftFront[2] = bar[0];
  leftBack[1] = leftFront[1] = bar[1];
  leftBack[0] = leftFront[0] = bar[2];

  rightBack[0] = rightFront[0] = bar[3];
  rightBack[1] = rightFront[1] = bar[4];
  rightBack[2] = rightFront[2] = bar[5];

}

// adjust lights on the down/legs with frequency readings
void updateLegsByFrequency() {
  // crappy
  uint16_t avgPower[N_SENSOR] = {0};
  // frequency power
  uint16_t power[N_SENSOR][N_FREQ_BINS] = {{0.0}};

  // do the boneheaded thing and sum up the bins across all sensors
  uint32_t sumSensors[N_FREQ_BINS] = {0};
  uint32_t maxSum = 0;
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      sumSensors[j] += sAF[myArch].freq.power[i][j];
    }
    if ( sumSensors[j] > maxSum ) maxSum = sumSensors[j];
  }

  Serial << F("Freq bins: ");
  // set the LEDs proportional to bins, normalized to maximum bin
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
    uint32_t value = map( sumSensors[j],
                          (uint32_t)0, maxSum,
                          (uint32_t)0, (uint32_t)255
                        );
    Serial << value << ",";
    leftDown[j] = CHSV(archHue[myArch], archSat[myArch], brighten8_video(constrain(value, 0, 255)));
  }
  Serial << endl;

  rightDown = leftDown;
}

// adjust lights on down/legs with peak frequency readings
void updateLegsByPeakFreq() {

  // some constants
  // maximum possible frequency bin
  const byte maxFreq = ceil( (N_FREQ_BINS+1)*DISTANCE_SAMPLING_FREQ/N_FREQ_SAMPLES );
  const byte minFreq = floor( (0+1)*DISTANCE_SAMPLING_FREQ/N_FREQ_SAMPLES );
  const byte fadeEachUpdate = 128;
  
  // fade lighting
  leftDown.fadeToBlackBy( fadeEachUpdate );

  // loop across each arch's information
  for( byte a=0; a<N_ARCH; a++ ) {
    // loop across each sensor
    for( byte s=0; s<N_SENSOR; s++ ) {
      // get the integer frequency above and below
      byte roundUp = ceil(sAF[a].freq.peakFreq[s]);
      byte roundDown = floor(sAF[a].freq.peakFreq[s]);
      
      // map to lighting position
      byte lightUp = map(roundUp, minFreq, maxFreq, 0, LEDS_VERT-1);
      byte lightDown = map(roundDown, minFreq, maxFreq, 0, LEDS_VERT-1);
      
      // pick a color; my color is the brightest.
      CHSV color = CHSV(archHue[a], 255, a == myArch ? 255 : 128 );

      // apply
      leftDown[lightUp] += color;
      leftDown[lightDown] += color;
    }
    
  }

  // What immortal hand or eye / Could frame thy fearful symmetry?
  rightDown = leftDown;
}

// update lights on the up/overhead with concordance readings
void updateTopsByConcordance() {
  // NOP
}

// show
void pushToHardware() {
  FastLED.show();

  // show our frame rate, periodically
  EVERY_N_SECONDS( 20 ) {
    uint16_t reportedFPS = FastLED.getFPS();
    Serial << F("FastLED reported FPS, Hz=") << reportedFPS << endl;
  }
}

// storage
void fillBuffer() {
  // track the buffer index
  static uint16_t index = 0;

  // push to buffer
  for ( byte i = 0; i < N_SENSOR; i++ ) buffer[i][index] = dist.prox[i];

  // increment buffer index
  index++;

  if ( index >= N_FREQ_SAMPLES ) {
    index = 0;
    bufferReady = true;
  }
}

// show sensor readings
void dumpBuffer(uint16_t count) {
  const char sep = ',';

  for ( uint16_t j = 0; j < N_FREQ_SAMPLES; j++) {
    for ( uint16_t i = 0; i < N_SENSOR; i++ ) {
      Serial << (uint16_t)buffer[i][j] << sep;
    }
    Serial << count << endl;
  }
  count ++;
}

// frequency calculations readings
void dumpFFT() {
  const char sep = ',';
  unsigned long now = millis();

  for ( uint16_t i = 0; i < N_SENSOR; i++ ) {
    Serial << now << sep << i << sep << sAF[myArch].freq.avgPower[i];
    for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
      Serial << sep << sAF[myArch].freq.power[i][j];
    }
    Serial << endl;
  }
}


// https://cdn-learn.adafruit.com/downloads/pdf/fft-fun-with-fourier-transforms.pdf
// Nyquist's Sampling Theorem: only frequencies up to half the sampling rate can
// can be detected.  So, for 33 fps (Hz) we can detect signals no faster than 33/2 Hz.
/*
  Finally, the output of the FFT on real data has a few interesting properties:
    The very first bin (bin zero) of the FFT output represents the average power of the signal.
  Be careful not to try interpreting this bin as an actual frequency value!
   Only the first half of the output bins represent usable frequency values. This means the
  range of the output frequencies detected by the FFT is half of the sample rate. Don't try to
  interpret bins beyond the first half in the FFT output as they won't represent real frequency
  values!
*/
double imag[N_FREQ_SAMPLES];
double real[N_FREQ_SAMPLES];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
void computeFFT(byte index) {
  // track time
  unsigned long tic = millis();

  // crunch the buffer; expensive
  const uint8_t exponent = FFT.Exponent(N_FREQ_SAMPLES); // can precompute to save a little time.

  // storage.  fft computations alter the buffer, so we copy
  for ( uint16_t j = 0; j < N_FREQ_SAMPLES ; j++ ) {
    real[j] = (double)buffer[index][j];
    imag[j] = 0.0;
  }

  // weigh data
  FFT.Windowing(real, N_FREQ_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //    Serial.println("Weighed data:"); PrintVector(real, N_FREQ_SAMPLES, SCL_TIME);

  // compute FFT
  FFT.Compute(real, imag, N_FREQ_SAMPLES, exponent, FFT_FORWARD); /* Compute FFT */
  //    Serial.println("Computed Real values:"); PrintVector(real, N_FREQ_SAMPLES, SCL_INDEX);
  //    Serial.println("Computed Imaginary values:"); PrintVector(imag, N_FREQ_SAMPLES, SCL_INDEX);

  // compute magnitudes
//  FFT.ComplexToMagnitude(real, imag, N_FREQ_SAMPLES); /* Compute magnitudes */
  // stealing a march here, as we don't need the magnitude information for the latter
  // half of the array
  FFT.ComplexToMagnitude(real, imag, N_FREQ_BINS + 2); /* Compute magnitudes */
//  Serial.println("Computed magnitudes:");  PrintVector(real, (N_FREQ_SAMPLES >> 1), SCL_FREQUENCY);

  // find major frequency peak
  double peakFreq = FFT.MajorPeak(real, N_FREQ_SAMPLES, DISTANCE_SAMPLING_FREQ);
//  if( index==5 ) Serial << F("Peak: ") << x << endl;
  
  // store power magnitudes of the lowest N_FREQ_BINS in the spectra
  sAF[myArch].freq.avgPower[index] = (uint16_t)real[0];
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) sAF[myArch].freq.power[index][j] = (uint16_t)real[j + 1];

  // store peak frequency
  sAF[myArch].freq.peakFreq[index] = (float)peakFreq;
  
/*
  unsigned long dur = (millis() - tic);
  if ( index == 0 ) {
    Serial << F("Last FFT complete.");
    Serial << F(" duration=") << dur;
    Serial << F(" ms.") << endl;
  }
  */
}
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++)   {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / DISTANCE_SAMPLING_FREQ);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * DISTANCE_SAMPLING_FREQ) / (double)N_FREQ_SAMPLES);
        break;
    }
    Serial.print(abscissa, 6);
    Serial.print(" ");
    Serial.print(vData[i], 4);
    Serial.println();

    // Only the first half of the output bins represent usable frequency values.
    if ( scaleType == SCL_FREQUENCY && i >= (bufferSize / 2 - 1) ) return;
  }
  Serial.println();
}

/*
void getConcordance() {

  // this is all wrong.  need 2d correlation analysis
  // https://en.wikipedia.org/wiki/Two-dimensional_correlation_analysis

  // maybe we want to push this to a separate microcontroller, Sepal_Concordance
  // alternately, each Arch can calculate their concordance clockwise (or ccw) and publish
  // that

  // normalize weights to sum to 1.0
  static float vecWeight[N_FREQ_BINS] = {0.0};
  // scale covariance terms
  static float covTerm = 0.0;

  // we can compute a few things once.
  static float isInitialized = false;
  if ( ! isInitialized ) {
    float sumWeight = 0;
    for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
      sumWeight += WeightFrequency[b];
    }
    for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
      vecWeight[b] = (float)WeightFrequency[b] / sumWeight;
      covTerm += vecWeight[b] * vecWeight[b];
    }
    covTerm = 1.0 / (1.0 - covTerm);
    isInitialized = true;
  }

  // weighted mean vector
  float xBar[N_ARCH] = {0.0};
  for ( byte j = 0; j < N_ARCH; j++ ) {
    for ( byte i = 0; i < N_FREQ_BINS; i++ ) {
      // this doesn't look quite right:
      xBar[j] += vecWeight[j] * (float)sAF[myArch].freq.power[i][j];
    }
  }

  // weighted covariance matrix
  float Cov[N_ARCH][N_ARCH] = {0.0};
  for ( byte a1 = 0; a1 < N_ARCH; a1++ ) {
    for ( byte a2 = a1; a2 < N_ARCH; a2++ ) { // symmetry
      float sum = 0.0;
      for ( byte b = 0; b < N_FREQ_BINS; b++ ) {
        sum += vecWeight[b] *
               ((float)sAF[a1].freq[b] - xBar[a1]) *
               ((float)sAF[a2].freq[b] - xBar[a2]);
      }
      Cov[a1][a2] = covTerm * sum;
    }
  }

  // weighted correlation matrix
  float Cor[N_ARCH][N_ARCH] = {0.0};
  float Is[N_ARCH];
  for ( byte a = 0; a < N_ARCH; a++ ) {
    Is[a] = 1.0 / pow(Cov[a][a], 0.5);
  }
  for ( byte a1 = 0; a1 < N_ARCH; a1++ ) {
    for ( byte a2 = a1; a2 < N_ARCH; a2++ ) { // symmetry
      if ( a1 != a2 ) Cor[a1, a2] = Is[a1] * Cov[a1][a2] * Is[a2];
    }
  }

  // save
  switch ( myArch ) {
    case 0: // A. B is next. C is prev.
      concordNext = Cor[0, 1]; // corr.AB = mat.cor[1,2]
      concordPrev = Cor[0, 2]; // corr.CA = mat.cor[1,3]
      break;
    case 1: // B. C is next. A is prev.
      concordNext = Cor[1, 2]; // corr.BC = mat.cor[2,3]
      concordPrev = Cor[0, 1]; // corr.AB = mat.cor[1,2]
      break;
    case 2: // C. A is next. B is prev.
      concordNext = Cor[0, 2]; // corr.CA = mat.cor[1,3]
      concordPrev = Cor[1, 2]; // corr.BC = mat.cor[2,3]
      break;
  }
  concordTotal = (Cor[0, 1] + Cor[1, 2] + Cor[0, 2]) / 3.0;

}
*/
