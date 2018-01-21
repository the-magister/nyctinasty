// IDE Settings:
// Tools->Board : "WeMos D1 R2 & mini"
// Tools->Flash Size : "4M (3M SPIFFS)"
// Tools->CPU Frequency : "160 MHz"
// Tools->Upload Speed : "921600"
/*

  Welcome to C++.  It's not a very friendly place, admittedly, but I'll do everything I can to
  sweep the gnarly mechanical stuff under the rug.

  We want Nyctinasty to be meaningfully different day-by-day.  Thus, I'm thinking about using
  color palettes to help us do that.  If we're going to mimic a plant growth season, and
  compress the season into four days, then we want:

  Sunday = Winter = cool, blues?, glare
  Monday = Spring = cool, greens?, pastels
  Tuesday = Summer = warm, yellows?, saturated
  Wedneday = Fall = hot, oranges?, saturated

  Probably, we'll try to burn the thing on Thursday night. If we burn Friday night, then we'll
  just set Thursday = Winter.

  So, what we need are four color palettes that reflect these characteristics.  With the test
  rig in your hands, you'll be able to see the palette choices.

  Some desireable palette properties:

  - 16 colors.  In practice, these 16 colors are interpolated to form 256 colors, so think
    of these 16 colors as "support points" for a larger set of colors.

  - Cyclical.  We'd like there to be a smooth transition between the last color of the
    palette and the first. A palette that's (Red, Orange, Yellow) is bad, because the "wrap"
    around from Yellow back to Red is abrupt.  (Red, Orange, Yellow, Orange) is correct, as
    we "wrap" smoothly.  However, you're going to get 50% Orange and only 25% Red and Yellow
    each.  (Red, Red, Orange, Yellow, Yellow, Orange) would yield 33% of each

*/

// Some tech-speak for "use some stuff that's already written".  Sadly, I must have this stuff first.
#include <Streaming.h>
#include <Metro.h>
#include <FastLED.h>

// color correction ("gamma correction")
// the apparent brightness of "red" is far lower than "green" or "blue".  So, we
// decrease the blue and green elements to compensate, running red at full.
// **CHOOSE ONE**
//#define COLOR_CORRECTION TypicalLEDStrip // 0xFFB0F0
//#define COLOR_CORRECTION Typical8mmPixel // 0xFFE08C
#define COLOR_CORRECTION UncorrectedColor // 0xFFFFFF
// or, roll your own hex code: rrggbb.  "FF" is full (16), which is what red should be set to.
//#define COLOR_CORRECTION 0xFFB0F0

// temperature correction ("white correction")
// I've never messed with this, but could be important
// **CHOOSE ONE**
//#define COLOR_TEMPERATURE Candle // =0xFF9329
//#define COLOR_TEMPERATURE Tungsten40W // =0xFFC58F
//#define COLOR_TEMPERATURE Tungsten100W // =0xFFD6AA
//#define COLOR_TEMPERATURE Halogen // =0xFFF1E0,
//#define COLOR_TEMPERATURE CarbonArc // =0xFFFAF4
//#define COLOR_TEMPERATURE HighNoonSun // =0xFFFFFB
//#define COLOR_TEMPERATURE DirectSunlight // =0xFFFFFF
//#define COLOR_TEMPERATURE OvercastSky // =0xC9E2FF
//#define COLOR_TEMPERATURE ClearBlueSky // =0x409CFF
//#define COLOR_TEMPERATURE WarmFluorescent // =0xFFF4E5
//#define COLOR_TEMPERATURE StandardFluorescent // =0xF4FFFA
//#define COLOR_TEMPERATURE CoolWhiteFluorescent // =0xD4EBFF
//#define COLOR_TEMPERATURE FullSpectrumFluorescent // =0xFFF4F2
//#define COLOR_TEMPERATURE GrowLightFluorescent // =0xFFEFF7
//#define COLOR_TEMPERATURE BlackLightFluorescent // =0xA700FF
//#define COLOR_TEMPERATURE MercuryVapor // =0xD8F7FF
//#define COLOR_TEMPERATURE SodiumVapor // =0xFFD1B2
//#define COLOR_TEMPERATURE MetalHalide // =0xF2FCFF
//#define COLOR_TEMPERATURE HighPressureSodium // =0xFFB74C
#define COLOR_TEMPERATURE UncorrectedTemperature // =0xFFFFFF 

// palette definitions

// you might visit this site to get access to a trove of palettes:
// http://fastled.io/tools/paletteknife/

// you can define colors using RGB or HSV codes.
// https://github.com/FastLED/FastLED/wiki/Pixel-reference

// we just need 16 "support" points for the color palette, defined for each of the four palettes

const CRGBPalette16 WinterPalette(
  CRGB::Blue,   // don't forget the comma after each entry
  CRGB::DarkBlue,
  CRGB::LightBlue,
  CRGB::White,

  CRGB::LightBlue,
  0x00008B, // which is the same as saying CRGB::DarkBlue, if you want to use the hex code
  CRGB::DarkBlue,
  CRGB::DarkBlue,

  CHSV(HUE_BLUE, 255, 255), // which is the same as saying CRGB::Blue, if you want to use the HSV code
  CRGB::DarkBlue,
  CRGB::SkyBlue,
  CRGB::SkyBlue,

  CRGB::LightBlue,
  CRGB::White,
  CRGB::LightBlue,
  CRGB::SkyBlue   // no comma after the 16th entry
);
const CRGBPalette16 SpringPalette(
  CRGB::LightSkyBlue,
  CRGB::Teal,
  CRGB::MidnightBlue,
  CRGB::Navy,

  CRGB::CornflowerBlue,
  CRGB::MediumBlue,
  CRGB::SeaGreen,
  CRGB::Teal,

  CRGB::CadetBlue,
  CRGB::Blue,
  CRGB::DarkCyan,
  CRGB::CornflowerBlue,

  CRGB::Aquamarine,
  CRGB::SeaGreen,
  CRGB::Aqua,
  CRGB::LightSkyBlue
);
const CRGBPalette16 SummerPalette(
  CRGB::Green,
  CRGB::DarkGreen,
  CRGB::DarkOliveGreen,
  CRGB::DarkGreen,

  CRGB::Green,
  CRGB::ForestGreen,
  CRGB::OliveDrab,
  CRGB::Green,

  CRGB::SeaGreen,
  CRGB::MediumAquamarine,
  CRGB::LimeGreen,
  CRGB::YellowGreen,

  CRGB::LightGreen,
  CRGB::LawnGreen,
  CRGB::MediumAquamarine,
  CRGB::ForestGreen
);
const CRGBPalette16 FallPalette(
  CRGB::Black,
  CRGB::Maroon,
  CRGB::Black,
  CRGB::Maroon,

  CRGB::DarkRed,
  CRGB::Maroon,
  CRGB::DarkRed,
  CRGB::Red,

  CRGB::DarkRed,
  CRGB::DarkRed,
  CRGB::Red,
  CRGB::Orange,

  CRGB::White,
  CRGB::Orange,
  CRGB::Red,
  CRGB::DarkRed
);

// cycle palettes on an interval.  Set this to 600 for ten minutes, if you want to study the palette in detail
byte secondsBetweenPaletteChange = 60/4; // seconds

// start with this palette
byte startPalette = 0; // 0,1,2,3 are Winter,Spring,Summar,Fall respectively.

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!! EVERYTHING BELOW THIS LINE DOES NOT NEED TO BE CHANGED !!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// some definitions
#define LED_PIN     D8
#define BRIGHTNESS  255
#define LED_TYPE    WS2811
#define COLOR_ORDER RGB
#define COLOR_CORRECTION TypicalLEDStrip
#define NUM_LEDS 20

// The leds
CRGB leds[NUM_LEDS];

// stack the paletts
const CRGBPalette16 *palettes[] = {&WinterPalette, &SpringPalette, &SummerPalette, &FallPalette};
byte paletteIndex;

void setup() {
  Serial.begin(115200);

  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  LEDS.setCorrection(COLOR_CORRECTION);
  LEDS.setTemperature(COLOR_TEMPERATURE);
  LEDS.setBrightness(BRIGHTNESS);

  setPalette(startPalette);
}


void loop() {
  // periodically choose a new palette
  EVERY_N_SECONDS( secondsBetweenPaletteChange ) 
    setPalette(paletteIndex + 1);

  // update the lights
  EVERY_N_MILLISECONDS( 20 ) {
    fillLEDsFromPaletteColors();
  }

  // show it
  LEDS.show();
}

void setPalette(byte i) {
  paletteIndex = i % 4;

  Serial << F("Setting palette to ");
  switch (paletteIndex) {
    case 0: Serial << "Winter"; break;
    case 1: Serial << "Spring"; break;
    case 2: Serial << "Summer"; break;
    case 3: Serial << "Fall"; break;
  }
  Serial << F("[") << i << F("/") << paletteIndex << F("]") << endl;

  // ramp down so we note palette change
  for(int i=255; i>1; i--) {
    FastLED.setBrightness(i);
    FastLED.show();
    FastLED.delay(5);
  }
  FastLED.setBrightness(BRIGHTNESS);
}

void fillLEDsFromPaletteColors() {
//  byte bright = qadd8(
//    beatsin8( brightCycle, 0, 255, 0, 0 ),
//    16 // keep some brightness.
//  );

  static byte brightStart = 0;
  byte brightDelta = 256/NUM_LEDS;
    
  static byte colorStart = 0;
  byte colorDelta = 256/NUM_LEDS;
  
  for ( int i = 0; i < NUM_LEDS; i++) {
    byte color = colorStart+colorDelta*i;
    byte bright = qadd8(triwave8(brightStart+brightDelta*i), 16);
     
    leds[i] = ColorFromPalette( *palettes[paletteIndex], color, dim8_video(bright), LINEARBLEND);
  }
  colorStart ++;
  brightStart --;
}

