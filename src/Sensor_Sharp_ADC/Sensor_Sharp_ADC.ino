// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "Skein_Messages.h"

// poll the analog pins for this long
unsigned long sensorPollTime = 50UL; // ms

// store readings
SensorReading reading = defaultSensorReading;

// store settings
Command settings = defaultCommand;

// set reading threshold; readings lower than this are oor
const unsigned long reading3000mm = 86;
const unsigned long reading2500mm = 102;
const unsigned long reading2000mm = 127;
const unsigned long oorReading = reading3000mm;

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
  // connect AREF to +3.3v via 6.4 kOhm resistor
  // if 6.2 kOhm used, 1023 reading is 2.76V
  // if 6.8 kOhm used, 1023 reading is 2.72V

  // reset the interval
  sensorPollTime = 1000UL / settings.fps;
  Serial << F("Startup. Target fps=") << settings.fps << endl;
  Serial << F("Startup. Sensor poll time (ms)=") << sensorPollTime << endl;

  // save reading information
  reading.min = 0;
  reading.max = readingToDistance(oorReading);
  reading.noise = readingToDistance(reading2000mm);
}

void loop() {
  // record the time
  unsigned long tic = millis();
  
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

  // do we need to adjust the poll time to get our target fps?
  unsigned long elapsed = millis() - tic;
  if( elapsed > (1000UL / settings.fps)+2 ) {
    sensorPollTime--;
    Serial << F("Adjusting sensor poll time down: ") << sensorPollTime << endl;
  } else if( elapsed < (1000UL / settings.fps)-2 ) {
    sensorPollTime++;
    Serial << F("Adjusting sensor poll time up: ") << sensorPollTime << endl;    
  }
}

void readSensors() {
  // create a timer
  Metro updateWhile(sensorPollTime);

  unsigned long updates = 0;
  static uint32_t smoothing = 100;
  static uint32_t values[] = {oorReading,oorReading,oorReading,oorReading,oorReading,oorReading,oorReading,oorReading};
  
  // get a bunch of readings
  while ( !updateWhile.check() ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      unsigned long r = analogRead(i);
      r = r < oorReading ? oorReading : r;
      values[i] = (values[i] * (smoothing - 1) + r) / smoothing;
    }
    updates ++;
  }
  smoothing = updates;

  // convert to ranges
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    reading.dist[i] = readingToDistance(values[i]);
  }
  
}

uint16_t readingToDistance( uint32_t reading ) {
  const uint32_t magicNumberSlope = 266371;
  const uint32_t magicNumberIntercept = 87;
  // prevent div0 or exceeding 16 bits of information
  reading = reading < oorReading ? oorReading : reading;
  
  uint32_t distance = magicNumberSlope / (uint32_t)reading - magicNumberIntercept;
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
  mySerial.flush();
}
