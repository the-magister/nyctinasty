#include <Streaming.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define NLOX 8
Adafruit_VL53L0X lox[NLOX] = {
  Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(),
  Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X() 
};
VL53L0X_RangingMeasurementData_t range[NLOX];

void selectLOX(uint8_t i) {
  const byte addr = 0x70;
  if (i > 7) return;
 
  Wire.beginTransmission(addr);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup(void)  {
  Serial.begin(115200);
  Serial << "Startup." << endl;
  
  // initialize the sensors
  for( byte i=0;i<NLOX;i++) {
    selectLOX(i);
    if( ! lox[i].begin() ) {
      Serial << "FAILED to initialize LOX " << i << endl;
    } else {
      Serial << "Initialized LOX " << i << endl;
    }
  }
}

void loop(void) {
  // track the time
  unsigned long now = millis();
  // poll the sensors
  for( byte i=0;i<NLOX;i++) {
    selectLOX(i);
    lox[i].rangingTest(&range[i], false);
    Serial << "Ranging " << i << ": ";
    
    if( range[i].RangeStatus !=4 ) {
      Serial << range[i].RangeMilliMeter << " mm." << endl;
    } else {
      Serial << "out of range." << endl;
    }
  }
  Serial << "Ranging Duration (ms): " << millis()-now << "\t fps (Hz):" << 1000/(millis()-now) << endl;
}
