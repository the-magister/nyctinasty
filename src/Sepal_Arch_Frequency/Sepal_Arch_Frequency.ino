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
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// wire it up
#define RX D5
#define TX D6
// voltage divider: 5V->3.3V
// TX to RX_ESP via 820 Ohm resistor
// RX_ESP to GND via 1500 Ohm resistor

// define a state for every systemState
void idleUpdate(); State Idle = State(idleUpdate);
void normalUpdate(); State Normal = State(normalUpdate);
State Reboot = State(reboot);
void reprogram() {
  reprogram("Sepal_Arch_Frequency.ino.bin");
}; State Reprogram = State(reprogram);
FSM stateMachine = FSM(Idle); // initialize state machine

// incoming message storage and flag for update
SystemCommand settings;     boolean settingsUpdate = false;

// our distance updates send as this structure as this topic
SepalArchDistance dist;     String distTopic;

// our frequency updates send as this structure as this topic
SepalArchFreq freq;         String freqTopic;

// FFT object
#define N_FREQ_SAMPLES (uint16_t)(1<<7)  // This value MUST ALWAYS be a power of 2
uint16_t buffer[N_SENSOR][N_FREQ_SAMPLES];
boolean bufferReady = false;
arduinoFFT FFT = arduinoFFT();

SoftwareSerial mySerial(RX, TX, false, 1024);
SoftEasyTransfer ETin;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << endl << F("Startup.") << endl;

  // for remote output
  mySerial.begin(115200);

  // messages
  ETin.begin(details(dist), &mySerial);

  // who am I?
  Id myId = commsBegin();

  // publish
  distTopic = commsTopicDistance(myId.sepal, myId.arch);
  freqTopic = commsTopicFrequency(myId.sepal, myId.arch);
  Serial << F("Publishing: ") << distTopic << endl;
  Serial << F("Publishing: ") << freqTopic << endl;

  // subscribe
  commsSubscribe(commsTopicSystemCommand(), &settings, &settingsUpdate, 1); // QoS 1
  
  Serial << F("Startup complete") << endl;
}

void loop() {
  // comms handling
  commsUpdate();

  // bail out if not connected
  if ( ! commsConnected() ) return;

  // check for settings update
  if ( settingsUpdate ) {
    switchState(settings.state);
    settingsUpdate = false;
  }

  // do stuff
  stateMachine.update();
}

void switchState(systemState state) {
  Serial << F("State.  Changing to ") << state << endl;
  switch ( state ) {
    case STARTUP: stateMachine.transitionTo(Idle); break;
    case NORMAL: stateMachine.transitionTo(Normal); break;
    case CENTRAL: stateMachine.transitionTo(Idle); break;
    case REBOOT: stateMachine.transitionTo(Reboot); break;
    case REPROGRAM: stateMachine.transitionTo(Reprogram); break;
    default:
      Serial << F("ERROR!  unknown state.") << endl;
  }
}

void idleUpdate() {
  // NOP; on hold until we hear from the Coordinator
}
void normalUpdate() {
  /*
     Interleave a FFT compute and publishing between distance updates every 10 ms.
     looks like:
     distance -> fft(sensor 0) -> distance -> fft(sensor 1) etc. then
     distance -> publish FFT
     distance -> distance etc.
  */

  // read ADCs
  if ( ETin.receiveData() ) {
    // ship it
    commsPublish(distTopic, &dist);

    // fill buffer
    fillBuffer();

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
    } else if ( publishReady ) {
      // noting that we don't publish on the same loop as a compute
      publishReady = false;
      // publish
      commsPublish(freqTopic, &freq);
    }

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
    Serial << now << sep << i << sep << freq.avgPower[i];
    for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) {
      Serial << sep << freq.power[i][j];
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
  freq.avgPower[index] = (uint16_t)real[0];
  for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) freq.power[index][j] = (uint16_t)real[j + 1];

  unsigned long toc = millis();
  if( index == 0 ) {
    Serial << F("FFT complete.  duration=") << toc - tic; 
    Serial << F(" ms.  receiving samples every=") << distanceSampleRate;
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

