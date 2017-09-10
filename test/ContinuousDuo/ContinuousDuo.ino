/* This example shows how to use continuous mode to take
  range measurements with the VL53L0X. It is based on
  vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

  The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>
#include <Streaming.h>

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed
//#define HIGH_SPEED
#define HIGH_ACCURACY

VL53L0X sensor1, sensor2;

void selectLOX(uint8_t i) {
  const byte addr = 0x70;
  if (i > 7) return;

  Wire.beginTransmission(addr);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  selectLOX(0);

  sensor1.init();
  sensor1.setTimeout(500);


#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor1.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor1.setMeasurementTimingBudget(100UL * 1000UL);
#endif

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor1.startContinuous(50);

  selectLOX(1);

  sensor2.init();
  sensor2.setTimeout(500);


#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor2.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor2.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor2.setMeasurementTimingBudget(100UL * 1000UL);
#endif
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor2.startContinuous(50);

}

void loop()
{
  selectLOX(0);
  static unsigned long avgRead1 = 0;
  static unsigned long avgRead2 = 0;
  const byte smoothing = 10;
  
  int read1 = sensor1.readRangeContinuousMillimeters();
  if (read1 > 2000) read1 = 2000;
  avgRead1 = (avgRead1*(smoothing-1)+read1)/smoothing;
  avgRead1 = read1;

  if (sensor1.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }


  selectLOX(1);
  int read2 = sensor2.readRangeContinuousMillimeters();
  if (read2 > 2000) read2 = 2000;
  avgRead2 = (avgRead2*(smoothing-1)+read2)/smoothing;
  avgRead2 = read2;

  if (sensor2.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }

//  Serial <<  read1 << "," << read2 << "," << avgRead1 << "," << avgRead2 << endl;
  Serial << avgRead1 << "," << avgRead2 << endl;
}
