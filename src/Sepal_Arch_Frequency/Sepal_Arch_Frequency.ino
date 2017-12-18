// compile for WeMos D1 R2 & mini
#include <Streaming.h>
#include <Metro.h>
#include <arduinoFFT.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include <FiniteStateMachine.h>
#include <ESP8266httpUpdate.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// define a state for every systemState
void idleUpdate(); State Idle = State(idleUpdate);
void normalUpdate(); State Normal = State(normalUpdate);
State Reboot = State(reboot);
void reprogram() {
  reprogram("Sepal_Arch_Frequency.ino.bin");
} State Reprogram = State(reprogram);
FSM stateMachine = FSM(Idle); //initialize state machine

// who am I?
const byte sepalNumber = 0;
const byte archNumber = 0;
const String id = commsIdSepalArchFrequency(sepalNumber, archNumber);

// ship settings
SystemCommand settings;
// in this topic
const String settingsTopic = commsTopicSystemCommand();
// and sets this true when an update arrives
boolean settingsUpdate = false;

// our distance updates send as this structure
SepalArchDistance dist;
// in this topic
const String distTopic = commsTopicDistance(sepalNumber, archNumber);

// our frequency updates send as this structure
SepalArchFreq freq;
// in this topic
const String freqTopic = commsTopicFreq(sepalNumber, archNumber);

// FFT object
#define N_FREQ_SAMPLES (uint16_t)(1<<8)  // This value MUST ALWAYS be a power of 2
double buffer[N_SENSOR][N_FREQ_SAMPLES];
arduinoFFT FFT = arduinoFFT();

// for comms
#define RX D5
#define TX D6
// voltage divider: 5V->3.3V
// TX to RX_ESP via 820 Ohm resistor
// RX_ESP to GND via 1500 Ohm resistor
SoftwareSerial mySerial(RX, TX, false, 1024);
SoftEasyTransfer ETin;

void setup() {
  // for local output
  Serial.begin(115200);

  Serial << endl << endl << F("Startup") << endl;

  // for remote output
  mySerial.begin(115200);

  // messages
  ETin.begin(details(dist), &mySerial);

  commsBegin(id);
  commsSubscribe(settingsTopic, &settings, &settingsUpdate, 1); // QoS 1
  Serial << F("Publishing: ") << distTopic << endl;
  Serial << F("Publishing: ") << freqTopic << endl;

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
  // read ADCs
  if ( ETin.receiveData() ) {
    // ship it
    dist.sepal = sepalNumber;
    dist.arch = archNumber;
    commsPublish(distTopic, &dist);

    // fill buffer
    if ( fillBuffer() ) {
      // dump the buffer
      //        Serial << endl;
      //        dumpBuffer();
      //        Serial << endl;

      // compute FFT
      computeFFT();

      // dump the average power for each sensor
      dumpFFT();

      // ship it
      commsPublish(freqTopic, &freq);
    }
  }

}

// storage
boolean fillBuffer() {
  // track the buffer index
  static uint16_t index = 0;

  // push to buffer
  for ( byte i = 0; i < N_SENSOR; i++ ) buffer[i][index] = dist.prox[i];

  // increment buffer index
  index++;

  if ( index >= N_FREQ_SAMPLES ) {
    index = 0;
    return ( true );
  }

  return ( false );
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


void computeFFT() {

  // crunch the buffer; expensive
  const uint8_t exponent = FFT.Exponent(N_FREQ_SAMPLES); // can precompute to save a little time.

  for ( byte i = 0; i < N_SENSOR; i++ ) {

    // storage for imaginary portion.
    double imag[N_FREQ_SAMPLES] = {0.0};

    // show buffer
    //    Serial.println("Data:"); PrintVector(vReal[i], samples, SCL_TIME);

    // weigh data
    FFT.Windowing(buffer[i], N_FREQ_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    //    Serial.println("Weighed data:"); PrintVector(buffer[i], samples, SCL_TIME);

    // compute FFT
    FFT.Compute(buffer[i], imag, N_FREQ_SAMPLES, exponent, FFT_FORWARD); /* Compute FFT */
    //    Serial.println("Computed Real values:"); PrintVector(buffer[i], samples, SCL_INDEX);
    //    Serial.println("Computed Imaginary values:"); PrintVector(vImag[i], samples, SCL_INDEX);

    // compute magnitudes
    //    FFT.ComplexToMagnitude(buffer[i], imag, N_FREQ_SAMPLES); /* Compute magnitudes */
    FFT.ComplexToMagnitude(buffer[i], imag, N_FREQ_BINS+2); /* Compute magnitudes */
    //    Serial.println("Computed magnitudes:");  PrintVector(buffer[i], (samples >> 1), SCL_FREQUENCY);

    // store power magnitudes
    freq.avgPower[i] = (uint16_t)buffer[i][0];
    for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) freq.power[i][j] = (uint16_t)buffer[i][j + 1];
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
