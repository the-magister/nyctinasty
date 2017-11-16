// compile for Nano
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>

// pin definitions
#define LED 13
#define RX 2
#define TX 3
// A0..A7 are also used
// wire 6.8kOhm resistor between +3.3 and AREF.

// for comms
SoftwareSerial mySerial(RX, TX); // cross pairs

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

}

void loop() {
  static boolean LEDstate = false;
  
  Metro readFor(500UL);
  readFor.reset();
  static unsigned long reading = 0;
  unsigned long count = 0;
  while( ! readFor.check() ) {
    unsigned long r = analogRead(0);
    reading += r;
    count++;
  }
  reading /= count;
    
  Serial << reading << "," << count << "," << analogRead(0) << endl;
  mySerial << reading << endl;
  LEDstate = !LEDstate;
  digitalWrite(LED, LEDstate);
}
