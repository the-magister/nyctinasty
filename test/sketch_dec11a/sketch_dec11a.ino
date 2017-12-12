void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogSetWidth(12); // sweet. 12-bit depth.
  analogSetAttenuation(ADC_6db); // gives full-scale voltage 2.2V, so we'll clip 2.75-2.2V
  analogSetCycles(8); // default=8

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(A14));
  delay(50);
}
