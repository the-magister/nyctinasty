// compile for WEMOS LOLIN32
#include <Streaming.h>
#include <Metro.h>
#include <arduinoFFT.h>
#include "Nyctinasty_Messages.h"
#include "Nyctinasty_Comms.h"

// who am I?
const byte SepalNumber = 0;
const byte archNumber = 0;
const String id = commsIdSepalArchMotion(SepalNumber, archNumber);

// ship settings
SystemCommand settings;
// in this topic
const String settingsTopic = commsTopicSystemCommand();
// and sets this true when an update arrives
boolean settingsUpdate = false;

// our distance updates send as this structure
SepalArchDistance dist;
// in this topic
const String distTopic = commsTopicDistance(SepalNumber, archNumber);

// our frequency updates send as this structure
SepalArchFreq freq;
// in this topic
const String freqTopic = commsTopicFreq(SepalNumber, archNumber);

// ADC on ESP32
// https://esp-idf.readthedocs.io/en/v2.0/api/peripherals/adc.html
unsigned int adc[N_SENSOR] = {
//   A4, A5, A6, A7, A17, A16, A15, A14
   A4, A5, A6, A7, A4, A5, A6, A7
// 32, 33, 34, 35, 27,   14,  12,  13   <-  labels on topside of board
};
// shit.  need to use ADC1 pins, as ADC2 conflicts with WiFi
// ADC1: 32, 34-39

// track cycle time
unsigned long targetCycleTime = 20UL; // ms

// FFT object
arduinoFFT FFT = arduinoFFT();

// compute maximum cycle time from target fps
unsigned long cycleTime(byte fps) {
  settings.fps = fps;

  unsigned long time = 1000UL / (unsigned long)settings.fps; // ms
  Serial << F("Settings.  fps=") << settings.fps << F(" cycle time=") << time << F(" ms") << endl;

  return ( time );
}

double buffer[N_SENSOR][N_FREQ_SAMPLES];
uint16_t smoothing = 4;
boolean fillBuffer() {
  // track the active pin
  static byte sensor = 0;
  static uint16_t index = 0;

  // read
  uint16_t reading = analogRead( adc[sensor] );
  uint32_t smoothed = ((uint32_t)dist.prox[sensor] * ((uint32_t)smoothing - 1) + (uint32_t)reading) / (uint32_t)smoothing;
  // smooth
  dist.prox[sensor] = (uint16_t)smoothed;
  
  // push to buffer
  buffer[sensor][index] = reading;

  // increment
  sensor++;
  if (sensor >= N_SENSOR) {
    sensor = 0;
    index++;
  }

  if ( index >= N_FREQ_SAMPLES ) {
    index = 0;
    return ( true );
  }

  return ( false );
}

// send sensor readings
void dumpBuffer() {
  static uint16_t count = 0;

  const char sep = ',';

  for ( uint16_t j = 0; j < N_FREQ_SAMPLES; j++) {
    for ( uint16_t i = 0; i < N_SENSOR; i++ ) {
      Serial << (uint16_t)buffer[i][j] << sep;
    }
    Serial << count << endl;
  }
  count ++;
}

void setup() {
  // for local output
  Serial.begin(115200);

  // configure ADC
  // C:\Program Files (x86)\Arduino\hardware\espressif\esp32\cores\esp32\esp32-hal-adc.h
  analogSetWidth(12); // sweet. 12-bit depth.
  analogSetAttenuation(ADC_6db); // gives full-scale voltage 2.2V, so we'll clip 2.75-2.2V
  analogSetCycles(8); // default=8

  targetCycleTime = cycleTime(settings.fps);

  commsBegin(id);
  commsSubscribe(settingsTopic, &settings, &settingsUpdate);
//  commsSubscribe(settingsTopic, &settings, settingsUpdate);
  Serial << F("Publishing: ") << distTopic << endl;
  Serial << F("Publishing: ") << freqTopic << endl;
  commsUpdate();

}

void loop() {
  // comms handling
  commsUpdate();

  // bail out if not connected
  if ( ! commsConnected() ) return;

  // order these by priority:

  // check for settings update
  if( settingsUpdate ) {
    targetCycleTime = cycleTime(settings.fps);
    settingsUpdate = false;
  }

  // read ADCs
  static uint32_t tic = millis();
  if ( fillBuffer() ) {

    double secElapsed = (double)(millis() - tic)/1000.0;
    freq.samplingFrequency = (double)N_FREQ_SAMPLES / secElapsed; // report the interval between two updates of the same sensor
    
//    Serial << _FLOAT(secElapsed,4) << " sec elapsed for one buffer fill.  samplingFrequency=" << _FLOAT(freq.samplingFrequency,4) << " Hz." << endl;
/*
    // dump the buffer
    Serial << endl;
    dumpBuffer();
    Serial << endl;

    // compute FFT
    computeFFT();

    // ship it
    commsPublish(freqTopic, &freq);
*/
    tic = millis();
  }
  
  // push distance information
  static Metro sendDistInterval(targetCycleTime);
  static uint16_t counts = 0; // counting the update cycles
  counts++;
  if ( sendDistInterval.check() ) {
    // ship it
    commsPublish(distTopic, &dist);
    
    for( byte i=0; i<N_SENSOR; i++ ) Serial << dist.prox[i] << ",";
    Serial << targetCycleTime;
    Serial << endl;
    
    sendDistInterval.interval(targetCycleTime);
    sendDistInterval.reset();

    // adjust smoothing to capture average reading over targetCycleTime
//    smoothing = counts/(uint16_t)N_SENSOR;
//    if( smoothing < 1 ) smoothing = 1;
    counts = 0;
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

    // storage
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
    FFT.ComplexToMagnitude(buffer[i], imag, N_FREQ_SAMPLES); /* Compute magnitudes */
    //    Serial.println("Computed magnitudes:");  PrintVector(buffer[i], (samples >> 1), SCL_FREQUENCY);

    // store power magnitudes
    freq.avgPower[i] = buffer[i][0];
    for ( uint16_t j = 0; j < N_FREQ_BINS ; j++ ) freq.power[i][j] = buffer[i][j+1];

    // need to give up some time for other threads
    yield();
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
        abscissa = ((i * 1.0) / freq.samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * freq.samplingFrequency) / (double)N_FREQ_SAMPLES);
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

