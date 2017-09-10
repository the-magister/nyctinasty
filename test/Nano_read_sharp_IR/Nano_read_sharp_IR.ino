// compile for Nano
#include <Streaming.h>
#include <Metro.h>

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  analogReference(EXTERNAL); // tune to 2.75V
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial << analogRead(0) << ",";
  Serial << analogRead(1) << ",";
  Serial << analogRead(2) << ",";
//  Serial << analogRead(3) << ",";

  Serial << endl;
  delay(20);
}
