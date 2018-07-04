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
NyctRole myRole = N_ROLES; // see Nyctinasty_Comms.h; set N_ROLES to pull from EEPROM
// Arch0, Arch1, Arch2 for bootstrapping a new controller.
byte myArch;
byte leftArch, rightArch;
byte leftCoord, rightCoord;

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
#define LEDS_DOWN 17
#define LEDS_UP 13
#define LEDS_DECK 4
#define LEDS_PER_PIN LEDS_BAR+LEDS_DOWN

CRGBArray<LEDS_PER_PIN> leftBack;
CRGBArray<LEDS_PER_PIN> rightBack;
CRGBArray<LEDS_PER_PIN> leftFront;
CRGBArray<LEDS_PER_PIN> rightFront;

// bars
CRGBSet rightBar1 = rightBack(0, LEDS_BAR-1);
CRGBSet rightBar2 = rightFront(0, LEDS_BAR-1);
CRGBSet leftBar1 = leftBack(0, LEDS_BAR-1);
CRGBSet leftBar2 = leftFront(0, LEDS_BAR-1);

// verticals
CRGBSet leftUp = leftFront(LEDS_BAR, LEDS_UP-1);
CRGBSet rightUp = rightFront(LEDS_BAR, LEDS_UP-1);
CRGBSet leftDown = leftBack(LEDS_BAR, LEDS_DOWN-1);
CRGBSet rightDown = rightBack(LEDS_BAR, LEDS_DOWN-1);

// deck lights under rail
CRGBSet leftDeck = leftUp(LEDS_BAR+LEDS_UP, LEDS_PER_PIN-1);
CRGBSet rightDeck = rightUp(LEDS_BAR+LEDS_UP, LEDS_PER_PIN-1);

// color choices, based on arch information
const CHSV archColor[N_ARCH] = {
  CHSV(HUE_RED, 255, 255),
  CHSV(HUE_GREEN, 255, 255),
  CHSV(HUE_BLUE, 255, 255)
};

// deck lighting
CRGB deckColor = CRGB::FairyLight;

// comms
NyctComms comms;

// define a state for every systemState
void startup(); State Startup = State(startup);
void lonely(); State Lonely = State(lonely);
void ohai(); State Ohai = State(ohai);
void goodnuf(); State Goodnuf = State(goodnuf);
void goodjob(); State Goodjob = State(goodjob);
void winning(); State Winning = State(winning);
void fanfare(); State Fanfare = State(fanfare);
void reboot() { comms.reboot(); }; State Reboot = State(reboot);
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
  switch(myArch) {
    case 0: 
      // sC.isPlayer indexes
      leftArch=1; rightArch=2; 
      // sC.areCoordinated indexes
      leftCoord=0; rightCoord=2;
      break;
    case 1: 
      leftArch=2; rightArch=0; 
      leftCoord=1; rightCoord=0;
      break;
    case 2: 
      leftArch=0; rightArch=1; 
      leftCoord=2; rightCoord=1;
      break;    
  }
  Serial << F("Arch indexes: left=") << leftArch << F(" my=") << myArch << F(" right=") << rightArch << endl;
  Serial << F("Coordination indexes: left=") << leftCoord << F(" right=") << rightCoord << endl;
  
  // subscribe
  comms.subscribe(&sC.settings, &sC.hasUpdate);
  for ( byte a = 0; a < N_ARCH; a++ ) {
    // no need to subscribe to our own message
    if ( a != myArch ) comms.subscribe(&sAF[a].freq, &sAF[a].hasUpdate, a);
  }

  Serial << F("DISTANCE_SAMPLING_RATE, ms: ") << DISTANCE_SAMPLING_RATE << endl;
  Serial << F("DISTANCE_SAMPLING_FREQ, Hz: ") << DISTANCE_SAMPLING_FREQ << endl;
  Serial << F("N_FREQ_SAMPLES, #: ") << N_FREQ_SAMPLES << endl;
  Serial << F("FILL_TIME, ms: ") << FILL_TIME << endl;
  Serial << F("Lowest Freq Bin, Hz: ") << (float)(0+1)*(float)DISTANCE_SAMPLING_FREQ/(float)N_FREQ_SAMPLES << endl;
  Serial << F("Highest Freq Bin, Hz: ") << (float)(N_FREQ_BINS+1)*(float)DISTANCE_SAMPLING_FREQ/(float)N_FREQ_SAMPLES << endl;

  Serial << F("Startup complete.") << endl;
}

void loop() {
  // comms handling
  comms.update();

/*
  // after 5 seconds, transition to Offline, but we could easily get directed to Online before that.
  static Metro cycleTimeout(5000UL);
  if( cycleTimeout.check() ) {
    sC.settings.state = (systemState)((int)sC.settings.state+1);
    if( sC.settings.state == REBOOT ) sC.settings.state = STARTUP;
    sC.hasUpdate = true;
    cycleTimeout.reset();
  }
*/

  // check for settings update
  if ( sC.hasUpdate ) {
    updateState();
    calculateCoordPalette();
    calculatePlayerPalette();
    sC.hasUpdate = false;
  }

  // do stuff
  stateMachine.update();

  // update the sensor data
  updateSensors();
}

CRGBPalette16 coordPalette;
void calculateCoordPalette() {
  
  CRGBArray<16> pal;
  
  if( sC.settings.areCoordinated[leftCoord] ) {
    pal.fill_gradient(archColor[leftArch], archColor[myArch], archColor[leftArch] );
  } else {
    pal = CRGB::Black;
  }
  if( sC.settings.areCoordinated[rightCoord] ) {
    pal.fill_gradient(archColor[rightArch], archColor[myArch], archColor[rightArch] );
  } else {
    pal = CRGB::Black;
  }

  // copy out
  for(byte i=0; i<16; i++) coordPalette[i] = pal[i];
}

CRGBPalette16 playerPaletteLeft, playerPaletteRight; // left, right
void calculatePlayerPalette() {
  
  CHSV me = CHSV(archColor[myArch].hue, archColor[myArch].sat, 
    sC.settings.isPlayer[myArch] ? 255 : 0
  );
  CHSV left = CHSV(archColor[leftArch].hue, archColor[leftArch].sat, 
    sC.settings.isPlayer[leftArch] ? 255 : 0
  );
  CHSV right = CHSV(archColor[rightArch].hue, archColor[rightArch].sat, 
    sC.settings.isPlayer[rightArch] ? 255 : 0
  );

  CRGBArray<16> palLeft;
  palLeft.fill_gradient(left, me, left);

  CRGBArray<16> palRight;
  palRight.fill_gradient(right, me, right);
 
  // copy out
  for(byte i=0; i<16; i++) {
    playerPaletteLeft[i] = palLeft[i];
    playerPaletteRight[i] = palRight[i];
  }
}


void updateState() {

  switch ( sC.settings.state ) {
    case STARTUP: stateMachine.transitionTo(Startup); break;
    case LONELY: stateMachine.transitionTo(Lonely); break;
    case OHAI: stateMachine.transitionTo(Ohai); break;
    case GOODNUF: stateMachine.transitionTo(Goodnuf); break;
    case GOODJOB: stateMachine.transitionTo(Goodjob); break;
    case WINNING: stateMachine.transitionTo(Winning); break;
    case FANFARE: stateMachine.transitionTo(Fanfare); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }

  Serial << F("State change. to=");
  switch( sC.settings.state ) {
    case STARTUP: Serial << "STARTUP"; break;  //  all roles start here
  
    case LONELY: Serial << "LONELY"; break;   // 0 players
    case OHAI: Serial << "OHAI"; break;   // 1 players
    case GOODNUF: Serial << "GOODNUF"; break;  // 2 players
    case GOODJOB: Serial << "GOODJOB"; break;  // 3 players or 2 players coordinated
    case WINNING: Serial << "WINNING"; break;  // 3 players and 2 players coordinated
    case FANFARE: Serial << "FANFARE"; break;  // 3 players and 3 players coordinated 
  
    case REBOOT: Serial << "REBOOT"; break;   //  trigger to reboot 
  }
  Serial << ". isPlayer? A0=" << sC.settings.isPlayer[0];
  Serial << " A1=" << sC.settings.isPlayer[1];
  Serial << " A2=" << sC.settings.isPlayer[2];
  Serial << ". coord? A01=" << sC.settings.areCoordinated[0];
  Serial << " A12=" << sC.settings.areCoordinated[1];
  Serial << " A20=" << sC.settings.areCoordinated[2];
  Serial << endl;

}

void startup() {
  
  static byte hue = archColor[myArch].hue;
  EVERY_N_MILLISECONDS(10) {
    // show a throbbing rainbow background
    hue++;
    leftUp.fill_rainbow(hue, 255 / leftUp.size()); // paint
    rightUp = leftUp;
    leftDown.fill_rainbow(hue + 128, -255 / leftDown.size());
    rightDown = leftFront;
    
    // color the bar with our color
    leftBar1.fill_solid(archColor[myArch]);
    rightBar2 = rightBar1 = leftBar2 = leftBar1;
    
    // do it now
    pushToHardware();
  }

}

void lonely() {
  // need some kind of "walk into the arches" animation
  playerAndCoordinationUpdate();
}

void ohai() {
  playerAndCoordinationUpdate();
}

void goodnuf() {
  playerAndCoordinationUpdate();
}

void goodjob() {
  playerAndCoordinationUpdate();
}

void winning() {
  playerAndCoordinationUpdate();
}

void fanfare() {
  // need some kind of "yeah, playah!" animation
  playerAndCoordinationUpdate();
}

void playerAndCoordinationUpdate() {
  updateDeck();
  updateTopsByCoordination();
  updateLegsByPlayer();
}

void updateDeck() {
  leftDeck.fill_solid(deckColor);
  rightDeck.fill_solid(deckColor);
}

void updateTopsByCoordination() {
  static byte colorIndex = 0;  
  
  EVERY_N_MILLISECONDS( 20 ) {
    colorIndex ++;
    byte j = 0;
    for ( int i = 0; i < leftUp.size(); i++) {
      leftUp[i] = ColorFromPalette( coordPalette, colorIndex + j++, 255, LINEARBLEND );
    }
    j = 0;
    for ( int i = 0; i < rightUp.size(); i++) {
      rightUp[i] = ColorFromPalette( coordPalette, colorIndex + j++, 255, LINEARBLEND );
    }
  }  
}

void updateLegsByPlayer() {
  static byte colorIndex = 0;
  
  EVERY_N_MILLISECONDS( 20 ) {
    colorIndex ++;
    byte j = 0;
    for ( int i = 0; i < leftDown.size(); i++) {
      leftDown[i] = ColorFromPalette( playerPaletteLeft, colorIndex + j++, 255, LINEARBLEND );
    }
    j = 0;
    for ( int i = 0; i < rightDown.size(); i++) {
      rightDown[i] = ColorFromPalette( playerPaletteRight, colorIndex + j++, 255, LINEARBLEND );
    }
  }  

}

void updateSensors() {

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
 
 static uint32_t counter = 0;
  static boolean publishReady = false;
  static byte fftIndex = 0;
  static Metro pushDistanceInterval(distancePublishRate);

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
  // toggled TX pin
  static boolean pinState = false;
  static Metro distanceUpdate(DISTANCE_SAMPLING_RATE);

  if ( distanceUpdate.check() ) {
    distanceUpdate.reset();
    pinState = !pinState;
    digitalWrite(TX, pinState);
  }

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
      comms.publish(&sAF[myArch].freq, myArch);
      // flag that our frequency data are ready
      sAF[myArch].hasUpdate = true;
    } else if( pushDistanceInterval.check() ) {
      // publish distance
      comms.publish(&dist, myArch);
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

// adjust lights on the bar with distance readings
void updateBarByDistance() {

  // compute using a dummy set of LEDs
  static CRGBArray<N_SENSOR> bar;
  for ( byte s = 0; s < N_SENSOR; s++ ) {
    // quash noise
    if ( dist.prox[s] < dist.noise ) dist.prox[s] = dist.min;
    uint16_t intensity = map(
                           dist.prox[s],
                           dist.min, dist.max,
                           (uint16_t)0, (uint16_t)255
                         );
    bar[s] = CHSV(archColor[myArch].hue, archColor[myArch].sat, (byte)intensity);
  }

  // assign to hardware. ugly and direct, but we can see what's going on.
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
  for ( uint16_t b = 0; b < N_FREQ_BINS ; b++ ) {
    for ( byte s = 0; s < N_SENSOR; s++ ) {
      sumSensors[b] += sAF[myArch].freq.power[s][b];
    }
    if ( sumSensors[b] > maxSum ) maxSum = sumSensors[b];
  }

  Serial << F("Freq bins: ");
  // set the LEDs proportional to bins, normalized to maximum bin
  for ( uint16_t b = 0; b < N_FREQ_BINS ; b++ ) {
    uint32_t value = map( sumSensors[b],
                          (uint32_t)0, maxSum,
                          (uint32_t)0, (uint32_t)255
                        );
    Serial << value << ",";
    leftDown[b] = CHSV(archColor[myArch].hue, archColor[myArch].sat, brighten8_video(constrain(value, 0, 255)));
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
      byte lightUp = map(roundUp, minFreq, maxFreq, 0, LEDS_UP-1);
      byte lightDown = map(roundDown, minFreq, maxFreq, 0, LEDS_DOWN-1);
      
      // pick a color; my color is the brightest.
      byte bright = 128;
      if( a == myArch ) bright = 255;
      CHSV color = CHSV(archColor[a].hue, 255, bright );

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

double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


