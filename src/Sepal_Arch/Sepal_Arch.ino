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

// wire it up
// RX/TX for softserial comms
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
#define LEDS_BAR 4
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
void idle(); State Idle = State(idle);
void normal(); State Normal = State(normal);
void central(); State Central = State(central);
void reboot() {
  comms.reboot();
}; State Reboot = State(reboot);
void reprogram() {
  comms.reprogram("Sepal_Arch.ino.bin");
}; State Reprogram = State(reprogram);
FSM stateMachine = FSM(Idle); // initialize state machine
//FSM stateMachine = FSM(Normal); // initialize state machine

// my role
NyctRole role = Arch;

// incoming message storage and flag for update
struct sC_t {
  boolean hasUpdate = false;
  SystemCommand settings;
} sC;

// our distance updates send as this structure as this topic
SepalArchDistance dist;

// our frequency updates send as this structure as this topic
// other frequency updates receive as this structure as this topic
typedef struct {
  boolean hasUpdate = false;
  SepalArchFrequency freq;
} sAF_t;
sAF_t sAF[N_ARCH];

// FFT object
const uint32_t distanceSampleRate = 10; // ms
#define N_FREQ_SAMPLES (uint16_t)(1<<7)  // This value MUST ALWAYS be a power of 2
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
  runStartupPattern();
  Serial << F(" done.") << endl;

  // start comms
//  comms.begin(role, true);

  // subscribe
//  comms.subscribe(&sC.settings, &sC.hasUpdate);
//  for ( byte i = 0; i < N_ARCH; i++ ) {
//    // no need to subscribe to our own message
//    if ( i != comms.myArch() ) comms.subscribe(&sAF[i].freq, &sAF[i].hasUpdate, comms.mySepal(), i);
//  }

  Serial << F("Startup complete.") << endl;
  
  switchState(NORMAL);
}

void loop() {
  // comms handling
//  comms.update();

  // bail out if not connected
//  if ( ! comms.isConnected() ) return;

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
    case IDLE: stateMachine.transitionTo(Idle); break;
    case NORMAL: stateMachine.transitionTo(Normal); break;
    case CENTRAL: stateMachine.transitionTo(Central); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void idle() {
  static byte hue = 0;

  EVERY_N_MILLISECONDS(10) {
    // show a throbbing rainbow background
    hue++;
    leftBack.fill_rainbow(hue, 255 / leftBack.size()); // paint
    rightBack = leftBack;
    leftFront.fill_rainbow(hue + 128, -255 / leftFront.size());
    rightFront = leftFront;
    FastLED.show();
  }
}

void askForDistance() {
  // toggled TX pin
  static boolean pinState = false;
  static Metro distanceUpdate(distanceSampleRate);

  if ( distanceUpdate.check() ) {
    distanceUpdate.reset();
    pinState = !pinState;
    digitalWrite(TX, pinState);
  }
}

void normal() {
  // check for an update to concordance
  if ( sAF[0].hasUpdate && sAF[1].hasUpdate && sAF[2].hasUpdate ) {
    // are we concordant?
//    getConcordance();
    // show it
    updateLightsByConcordance();
    // reset
    sAF[0].hasUpdate = sAF[1].hasUpdate = sAF[2].hasUpdate = false;
  }

  // check to see if we need to pull new distance data.
  askForDistance();

  // get data from ADCs
  static uint32_t counter = 0;
  if ( ETin.receiveData() ) {
    // we need to update the LEDs now while we won't screw up SoftwareSerial and WiFi
    pushToHardware();

    counter ++;

    // fill buffer
    fillBuffer();

    // update bar lights
    updateLightsByDistance();
  }

  // do FFT in segements.
  static byte fftIndex = 0;
  static boolean publishReady = false;
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

  if ( publishReady ) {
    publishReady = false;
    // publish
//    comms.publish(&sAF[comms.myArch()].freq, comms.mySepal(), comms.myArch());
    // flag that our frequency data are ready
    sAF[comms.myArch()].hasUpdate = true;
    // update lights
    updateLightsByFrequency();
  }

  const uint32_t reportInterval = 10;
  EVERY_N_SECONDS( reportInterval ) {
    uint32_t actualDistanceSampleRate = (reportInterval*1000UL)/counter;

    Serial << F("Distance sample interval, actual=") << actualDistanceSampleRate;
    Serial << F(" hypothetical=") << distanceSampleRate;
    Serial << F(" ms.") << endl;
    
    counter=0;
  }
  
}
void central() {
  // NOP, currently.  Will look for lighting directions, either through UDP (direct) or MQTT (procedural).
}

void runStartupPattern() {
  // find the pins
  leftBack.fill_solid(CRGB::Purple);
  rightBack.fill_solid(CRGB::Aqua);
  leftFront.fill_solid(CRGB::Red);
  rightFront.fill_solid(CRGB::Blue);
  FastLED.show();
  delay(1000);

  leftBack.fill_solid(CRGB::Black);
  rightBack.fill_solid(CRGB::Black);
  leftFront.fill_solid(CRGB::Black);
  rightFront.fill_solid(CRGB::Black);
  FastLED.show();
  delay(333);
}

// adjust lights on the bar with distance readings
void updateLightsByDistance() {

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
//    bar[i] = CHSV(archHue[comms.myArch()], archSat[comms.myArch()], (byte)intensity);
    bar[i] = CHSV(archHue[0], archSat[0], (byte)intensity);
  }

  // assign to hardware. ugly and direct, but we can see what's going on.
  leftBack[3] = leftFront[3] = bar[0];
  leftBack[2] = leftFront[2] = bar[1];
  leftBack[1] = leftFront[1] = bar[2];
  leftBack[0] = leftFront[0] = bar[3];

  rightBack[0] = rightFront[0] = bar[4];
  rightBack[1] = rightFront[1] = bar[5];
  rightBack[2] = rightFront[2] = bar[6];
  rightBack[3] = rightFront[3] = bar[7];

}

// adjust lights on the down/legs with frequency readings
void updateLightsByFrequency() {
  // crappy
  uint16_t avgPower[N_SENSOR] = {0};
  // frequency power
  uint16_t power[N_SENSOR][N_FREQ_BINS] = {{0.0}};

  // do the boneheaded thing and sum up the bins across all sensors
  uint32_t sumSensors[N_FREQ_BINS] = {0};
  uint32_t maxSum = 0;
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      sumSensors[j] += sAF[comms.myArch()].freq.power[i][j];
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
    leftDown[j] = CHSV(archHue[0], archSat[0], brighten8_video(constrain(value, 0, 255)));
  }
  Serial << endl;

  rightDown = leftDown;
}

// update lights on the up/overhead with concordance readings
void updateLightsByConcordance() {
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
    Serial << now << sep << i << sep << sAF[comms.myArch()].freq.avgPower[i];
    for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
      Serial << sep << sAF[comms.myArch()].freq.power[i][j];
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
  //    Serial.println("Weighed data:"); PrintVector(buffer[i], samples, SCL_TIME);

  // compute FFT
  FFT.Compute(real, imag, N_FREQ_SAMPLES, exponent, FFT_FORWARD); /* Compute FFT */
  //    Serial.println("Computed Real values:"); PrintVector(buffer[i], samples, SCL_INDEX);
  //    Serial.println("Computed Imaginary values:"); PrintVector(vImag[i], samples, SCL_INDEX);

  // compute magnitudes
  //    FFT.ComplexToMagnitude(buffer[i], imag, N_FREQ_SAMPLES); /* Compute magnitudes */
  FFT.ComplexToMagnitude(real, imag, N_FREQ_BINS + 2); /* Compute magnitudes */
  //    Serial.println("Computed magnitudes:");  PrintVector(buffer[i], (samples >> 1), SCL_FREQUENCY);

  // store power magnitudes of the lowest N_FREQ_BINS in the spectra
  sAF[comms.myArch()].freq.avgPower[index] = (uint16_t)real[0];
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) sAF[comms.myArch()].freq.power[index][j] = (uint16_t)real[j + 1];

  unsigned long dur = (millis() - tic);
  if ( index == 0 ) {
    Serial << F("Last FFT complete.");
    Serial << F(" duration=") << dur;
    Serial << F(" ms.") << endl;
  }
}

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
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
        abscissa = ((i * 1.0) / distanceSampleRate);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * distanceSampleRate) / (double)N_FREQ_SAMPLES);
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
      xBar[j] += vecWeight[j] * (float)sAF[comms.myArch()].freq.power[i][j];
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
  switch ( coms.myArch() ) {
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
