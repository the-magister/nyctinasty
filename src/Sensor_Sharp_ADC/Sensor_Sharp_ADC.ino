// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "Skein_Messages.h"

// poll the analog pins for this long
unsigned long sensorPollTime = 20UL; // ms

// store readings
SensorReading reading = defaultSensorReading;

// store settings
Command settings = defaultCommand;

// set reading threshold; readings lower than this are oor
#define OOR reading2500mm
#define NOISE reading2000mm
const unsigned long reading3000mm = 86;
const unsigned long reading2500mm = 102;
const unsigned long reading2000mm = 127;

// for comms
SoftwareSerial mySerial(3, 2); // RX, TX; cross to other pair.

void setup() {
  // for local output
  Serial.begin(115200);

  // for remote output
  mySerial.begin(57600);

  // put your setup code here, to run once:
  analogReference(EXTERNAL); // tune to 2.75V
  // has 32K built-in resistor.
  // connect AREF to +3.3v via 6.8 kOhm resistor
  // if 6.2 kOhm used, 1023 reading is 2.76V
  // if 6.8 kOhm used, 1023 reading is 2.72V

  // reset the interval
  sensorPollTime = 1000UL / settings.fps;
  Serial << F("Startup. Target fps=") << settings.fps << endl;
  Serial << F("Startup. Sensor poll time (ms)=") << sensorPollTime << endl;

  // save reading information
  reading.min = 0;
  reading.max = readingToDistance(OOR);
  reading.noise = readingToDistance(NOISE);

}

void loop() {
  // poll sensors
  readSensors();

  // we have missing sensors
  reading.dist[4] = reading.max;
  reading.dist[5] = reading.max;
  reading.dist[6] = reading.max;
  reading.dist[7] = reading.max;

  // print the readings
  printRanges();

  // send message
  sendRanges();

  // record the time
  static unsigned long tic = millis();
  static unsigned long updates = 0;
  updates++;
  if( updates == 10 ) {
    unsigned long elapsed = millis() - tic;
//    Serial << F("Update interval.  actual=") << elapsed/updates << F(" target=") << 1000UL/settings.fps << endl;
    
    if( elapsed/updates > (1000UL/settings.fps+1) ) {
      Serial << F("sensor poll time DOWN 1 to: ") << --sensorPollTime << endl;
    }
    if( elapsed/updates < (1000UL/settings.fps-1) ) {
      Serial << F("sensor poll time UP 1 to: ") << ++sensorPollTime << endl;      
    }

    tic = millis();
    updates = 0;
  }

}

void readSensors() {
  // create a timer
  Metro updateWhile(sensorPollTime);
  updateWhile.reset();
  
  unsigned long updates = 0;
  static uint32_t smoothing = 3;
  static uint32_t values[N_SENSOR];
  
  // get a bunch of readings
  while ( !updateWhile.check() ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      unsigned long r = analogRead(i);
//      r = r < oorReading ? oorReading : r;
      values[i] = (values[i] * (smoothing - 1) + r) / smoothing;
    }
    updates ++;
  }
  if( smoothing != updates ) {
    Serial << "smoothing moved to:" << updates << endl;
    smoothing = updates;
  }
  
  // convert to ranges
  for ( byte i = 0; i < N_SENSOR; i++ ) {
//    reading.dist[i] = readingToDistance2(values[i], minValues[i], maxValues[i]);
//    reading.dist[i] = readingToDistance(map(values[i], minValues[i], maxValues[i], oorReading, 1023));
    reading.dist[i] = readingToDistance( values[i] );
  }
  
}

uint16_t readingToDistance2( float reading, float minReading, float maxReading ) {
  float invReading = 1.0/reading;
  float invMax = 1.0/maxReading;
  float invMin = 1.0/minReading;
  float invRet = mapfloat(invReading, invMin, invMax, 0.0, 1.0)-1.0;
  float ret = 3000.0 * invRet;

  int dig = 5;
  Serial << reading << "," << _FLOAT(invReading,dig) << "," << _FLOAT(invMin,dig) << "," << _FLOAT(invMax,dig);
  Serial << "," << _FLOAT(invRet,dig) << "," << _FLOAT(ret,dig) << endl;
  
  return( ret );
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max ) {
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_max;
}

uint16_t readingToDistance( uint32_t reading ) {
  const uint32_t magicNumberSlope = 266371;
  const uint32_t magicNumberIntercept = 87;
  // prevent div0 or exceeding 16 bits of information
  reading = reading < OOR ? OOR : reading;
//  reading = reading < 6 ? 6 : reading;
  
  uint16_t distance = magicNumberSlope / (uint32_t)reading - magicNumberIntercept;
  return( distance );
}

// print ranges
void printRanges() {
  const char sep = ',';
  
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    Serial << reading.dist[i]  << sep;
  }
  Serial << reading.min << sep << reading.max << sep << reading.noise;
  Serial << endl;
}

void sendRanges() {
  mySerial.print( "MSG" );
  mySerial.write( (byte*)&reading, sizeof(reading) );
//  mySerial.flush();
}
