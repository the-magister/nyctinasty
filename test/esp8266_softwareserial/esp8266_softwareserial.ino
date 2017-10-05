// compile for NodeMCU 1.0
#include <Streaming.h>
#include <Metro.h>
#include <SoftwareSerial.h>

// send string
String message;

//SoftwareSerial mySerial(D2, D1, false, 256); // D2, D1 / RX, TX; cross to other pair.
SoftwareSerial mySerial(D5, D1); // D2, D1 / RX, TX; cross to other pair.

void setup() {
  // for local output
  Serial.begin(115200);

  // for remote output
  mySerial.begin(57600);

   // with 16 bit resolution, we could have "65535," 8 times with a NUL at the end.
  message.reserve((5 + 1) * 8 + 1);

  Serial << endl << endl << "Startup." << endl;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
}
void loop() { // run over and over
  static boolean pinState = false;
  if ( mySerial.available() > 0 || Serial.available() > 0 ) {
    pinState != pinState;
    digitalWrite(LED_BUILTIN, pinState);    
    delay(5);
  }
  while (mySerial.available() > 0) {
    Serial.write(mySerial.read());
  }
  while (Serial.available() > 0) {
    Serial.write(Serial.read());
    delay(10);
  }

}

