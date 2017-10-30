// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "Skein_Messages.h"

#include <EasyTransfer.h>
//create two objects
EasyTransfer ETin, ETout; 

// poll the analog pins every interval
Metro updateWhile(50UL);

// store readings
SensorReading reading;

// store settings
Command settings;

// set reading threshold; readings lower than this are oor
const word threshold3000mm = 86;
const word threshold2500mm = 102;
const word threshold2000mm = 127;
const word threshold1500mm = 167;
const word threshold1000mm = 245;
const word threshold500mm = 454;
const word outOfRangeReading = threshold2500mm;

// convert oor reading to oor distance
const unsigned long magicNumberSlope = 266371;
const unsigned long magicNumberIntercept = 87;
const unsigned long outOfRangeDistance = magicNumberSlope / (unsigned long)outOfRangeReading - magicNumberIntercept;

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// send string
String message;

SoftwareSerial mySerial(3, 2); // RX, TX; cross to other pair.

void setup() {
  // for local output
  Serial.begin(115200);

  // for remote output
  mySerial.begin(57600);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(settings), &mySerial);
  ETout.begin(details(reading), &mySerial);

  // put your setup code here, to run once:
  analogReference(EXTERNAL); // tune to 2.75V

  // Could have a message "3000," eight times and a terminator
  message.reserve((4 + 1) * 8 + 1);

  // ADC prescalar
  // see: http://forum.arduino.cc/index.php?topic=6549.0
  // set prescale to 128
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);
  // prescalar 16
  //  sbi(ADCSRA,ADPS2);
  //  cbi(ADCSRA,ADPS1);
  //  cbi(ADCSRA,ADPS0);

  // load settings
  loadCommand(settings);
  // reset the interval
  updateWhile.interval(1000UL/settings.fps);
  Serial << "Startup. Target fps=" << settings.fps << endl;
  Serial << "Startup. Update interval (ms)=" << 1000/settings.fps << endl;
      
  // save reading information
  reading.min = 0;
  reading.max = outOfRangeDistance;
  reading.noise = 0.1 * float(outOfRangeDistance);
}

void loop() {
  // poll sensors
  readSensors();

  // translate readings
  calculateRanges();

  // we have missing sensors
  reading.dist[4] = outOfRangeDistance;
  reading.dist[5] = outOfRangeDistance;
  reading.dist[6] = outOfRangeDistance;
  reading.dist[7] = outOfRangeDistance;

  // check for settings
  checkCommands();
  
  // send message
  sendRanges();
}

void readSensors() {
  // reset timer
  updateWhile.reset();
  word updates = 0;
  static unsigned long smoothing = 10;

  // get a bunch of readings
  while ( !updateWhile.check() ) {
    for ( byte i = 0; i < N_SENSOR; i++ ) {
      unsigned long r = analogRead(i);
      r = r < outOfRangeReading ? outOfRangeReading : r;
      reading.dist[i] = (reading.dist[i] * (smoothing - 1) + r) / smoothing;
    }
    updates ++;
  }
  smoothing = updates;
}

void calculateRanges() {
  // calculate ranges
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    reading.dist[i] = magicNumberSlope / reading.dist[i] - magicNumberIntercept;
  }
}

void checkCommands() {

  // check for commands
  if( ETin.receiveData() ) {
    // reset the interval
    updateWhile.interval(1000UL/settings.fps);
    
    Serial << "Command. Target fps=" << settings.fps << endl;
    Serial << "Command. Update interval (ms)=" << 1000/settings.fps << endl;

    // store this.
    saveCommand(settings);
  }
   
}

void sendRanges() {
  // send the readings
  for ( byte i = 0; i < N_SENSOR; i++ ) {
    Serial << reading.dist[i]  << ',';
  }
  Serial << endl;

  ETout.sendData();
}
