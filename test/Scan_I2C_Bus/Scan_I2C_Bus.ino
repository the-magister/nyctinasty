/*
   TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino

   Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
*/

#include "Wire.h"

#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


// standard Arduino setup()
void setup()
{
  while (!Serial);
  delay(1000);

  Wire.begin();

  Serial.begin(115200);
  Serial.println("\nTCAScanner ready!");

  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    scan();
  }
  Serial.println("\ndone");
}

void scan()
{
  byte error, address;
  int nDevices;

  Serial.println("\tScanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    if (address == TCAADDR) continue;

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("\tI2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("\tUnknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("\tNo I2C devices found.");
  else {
    Serial.print("\t");
    Serial.print(nDevices);
    Serial.println(" found.");
  }
}

void loop()
{
}
