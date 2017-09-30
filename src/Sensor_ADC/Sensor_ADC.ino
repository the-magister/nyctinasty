// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>

// poll the analog pins every interval
const unsigned long updateInterval = 20UL; // ms

// number of analog pins
const byte Npins = 8;

// store readings
uint32_t reading[Npins];

// target bit depth; cannot exceed 16
byte analogBits = constrain(12, 10, 16);

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

  // with 16 bit resolution, we could have "65535," 8 times with a NUL at the end.
  message.reserve((5 + 1) * 8 + 1);

}

void loop() {
  // note start time of reading
  unsigned long then = millis();

  // read sensors
//  readSensors();
/*
  for( byte i=0; i<4; i++ ) {
    byte smoothing = 5;
    word r = analogRead(i);
    static word lloq = 10;
    r = r<lloq ? lloq : r;
    
    reading[i] = (reading[i]*(smoothing-1) + r)/smoothing;
    
//    Serial << reading[i] << ",";
  }
  */
//  Serial << "0,1023" << endl;

//  reading[0] = analogRead(0);
// reading[1] = analogRead(1);
//  reading[2] = analogRead(2);
//  reading[3] = analogRead(3);


  Metro updateWhile(updateInterval);
  updateWhile.reset();
  word updates = 0;
  for( byte i=0; i<Npins; i++ ) reading[i] = 0;
  while( !updateWhile.check() ) {
    // get a bunch of readings
    for( byte i=0; i<Npins; i++ ) reading[i] += analogRead(i);
    updates ++;
  }
  for( byte i=0; i<Npins; i++ ) reading[i] /= updates;
  
  // send message
  sendReadings();

}

// from: https://forum.arduino.cc/index.php?topic=109672.0
//    FILE: analogReadN.pde
//  AUTHOR: Rob Tillaart
//    DATE: 2012-05-10
// PUPROSE: higher precision analogRead()
// http://www.atmel.com/Images/doc8003.pdf
void readSensors() {
  // reset sensor values
  for ( byte i = 0; i < Npins; i++ ) reading[i] = 0;

  byte bits = constrain(analogBits, 10, 16) - 10;
  int samples = 1 << (bits << 1);

  // read the sensors
  for ( byte j = 0; j < samples; j++ ) {
    for ( byte i = 0; i < Npins; i++ ) {
      word r = analogRead(i);
      
      reading[i] += r;
    }
  }

  // apply bitshift
  for ( byte i = 0; i < Npins; i++ ) reading[i] >> bits;
}

void sendReadings() {
  const String delim = ",";

  message =         String(reading[0], DEC) +
            delim + String(reading[1], DEC) +
            delim + String(reading[2], DEC) +
            delim + String(reading[3], DEC) //+
//            delim + String(reading[4], DEC) +
//            delim + String(reading[5], DEC) +
//            delim + String(reading[6], DEC) +
//            delim + String(reading[7], DEC);
;
  Serial << "0,1024,";
  
  Serial << message << endl;

  mySerial << message << endl;
}


