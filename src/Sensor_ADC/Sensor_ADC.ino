// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>

// poll the analog pins every interval
const unsigned long updateInterval = 20UL; // ms
Metro updateWhile(updateInterval);

// number of analog pins
const byte Npins = 8;

// store readings
uint32_t reading[Npins];

// set reading threshold; readings lower than this are oor
const word threshold3000mm = 86;
const word threshold2500mm = 102;
const word threshold2000mm = 127;
const word outOfRangeReading = threshold2000mm;

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
  
  // put your setup code here, to run once:
  analogReference(EXTERNAL); // tune to 2.75V

  // Could have a message "3000," eight times and a terminator
  message.reserve((4 + 1) * 8 + 1);

  // ADC prescalar
  // set prescale to 128
  // see: http://forum.arduino.cc/index.php?topic=6549.0
  sbi(ADCSRA,ADPS2) ;
  sbi(ADCSRA,ADPS1) ;
  sbi(ADCSRA,ADPS0) ;

}

void loop() {
  // poll sensors
  readSensors();
  
  // send message
  sendReadings();
}

void readSensors() {
  // reset timer
  updateWhile.reset();
  word updates = 0;

  // reset readings
  for( byte i=0; i<Npins; i++ ) reading[i] = 0;

  // get a bunch of readings
  while( !updateWhile.check() ) {
    for( byte i=0; i<Npins; i++ ) {
        word r = analogRead(i);
        r = r<outOfRangeReading ? outOfRangeReading : r;
        reading[i] += r;
    }
    updates ++;
  }
  
  // divide by reading count
  for( byte i=0; i<Npins; i++ ) reading[i] /= updates;
}  

void sendReadings() {
  const char delim = ',';
  const char term = '\0';

  // send the readings
  for( byte i=0; i<Npins; i++ ) {
    Serial << reading[i] << delim;
    mySerial << reading[i] << delim;
  }

  // finish with OOR information and terminator
  Serial << outOfRangeReading << endl;
  mySerial << term << endl;
  
}


