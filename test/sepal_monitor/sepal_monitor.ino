/*********************************************************************
  This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

  This example is for a 64x48 size display using I2C to communicate
  3 pins are required to interface (2 I2C and one reset)

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada  for Adafruit Industries.
  BSD license, check license.txt for more information
  All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts\Picopixel.h>
#define FH 6 // six pixels per character
#define FW 4 // six pixels per character

// pin def: C:\Users\MikeD\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\variants\d1_mini
// SCL GPIO5 is D1
// SDA GPIO4 is D2
#define OLED_RESET 0  // GPIO0 is D3
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 48)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// sizes
const uint16_t H = SSD1306_LCDHEIGHT;

// vertex definitions
uint16_t legVert[6][3][2];
uint16_t distVert[3][3][2];

// active component tracking
//                         up0    down1   up2    down3  up4    down5
boolean isLegActive[] = {  true,  true,   true,  true,  true,  true};
//                         arch0          arch1         arch2
boolean isDistActive[] = { true,          true,         true};

boolean mistActive = false;
boolean fireActive = false;

void setup()   {
  Serial.begin(115200);

  // set up dimensions
  initializeVertices();

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  // init done

  // fonts
  display.setFont(&Picopixel);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // 90 degree rotation
  display.setRotation(1);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  Serial.println("Setup complete.");
  /*
    // draw a single pixel
    display.drawPixel(10, 10, WHITE);
    // Show the display buffer on the hardware.
    // NOTE: You _must_ call display after making any drawing commands
    // to make them visible on the display hardware!
    display.display();
    delay(2000);
    display.clearDisplay();

    // draw many lines
    testdrawline();
    display.display();
    delay(2000);
    display.clearDisplay();

    // draw rectangles
    testdrawrect();
    display.display();
    delay(2000);
    display.clearDisplay();

    // draw multiple rectangles
    testfillrect();
    display.display();
    delay(2000);
    display.clearDisplay();

    // draw mulitple circles
    testdrawcircle();
    display.display();
    delay(2000);
    display.clearDisplay();

    // draw a white circle, 10 pixel radius
    display.fillCircle(display.width()/2, display.height()/2, 10, WHITE);
    display.display();
    delay(2000);
    display.clearDisplay();

    testdrawroundrect();
    delay(2000);
    display.clearDisplay();

    testfillroundrect();
    delay(2000);
    display.clearDisplay();

    testdrawtriangle();
    delay(2000);
    display.clearDisplay();

    testfilltriangle();
    delay(2000);
    display.clearDisplay();

    // draw the first ~12 characters in the font
    testdrawchar();
    display.display();
    delay(2000);
    display.clearDisplay();

    // draw scrolling text
    testscrolltext();
    delay(2000);
    display.clearDisplay();

    // text display tests
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Hello, world!");
    display.setTextColor(BLACK, WHITE); // 'inverted' text
    display.println(3.141592);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print("0x"); display.println(0xDEADBEEF, HEX);
    display.display();
    delay(2000);
    display.clearDisplay();

    // miniature bitmap display
    display.drawBitmap(30, 16,  logo16_glcd_bmp, 16, 16, 1);
    display.display();
    delay(1);

    // invert the display
    display.invertDisplay(true);
    delay(1000);
    display.invertDisplay(false);
    delay(1000);
    display.clearDisplay();

    // draw a bitmap icon and 'animate' movement
    testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);
  */
}


void loop() {

  static unsigned long now = millis();
  static int updates = 0;

  // simulate activity
  if ( random(255) < 16 ) {
    byte index = random(6);
    isLegActive[index] = !isLegActive[index];
  }
  if ( random(255) < 16 ) {
    byte index = random(3);
    isDistActive[index] = !isDistActive[index];
  }

  // show status; about 40 ms to show this update.
  display.clearDisplay();

  // show the online/offline systems
  updateActive();

  // show mister, etc. status
  updateEffects();

  // show time and environment
  updateTimeEnv();

  display.setCursor(0, H + FH);
  display.println("A reasonable amount of space to display.");

  // push
  display.display();

  updates++;
  if ( millis() - now > 10000UL ) {
    Serial.println(updates);
    updates = 0;
    now = millis();
  }
}

void updateEffects() {
  display.setCursor(0, FH+2);
  if( mistActive ) display.print("MIST");
  else display.print("mist");

  display.setCursor(H-4*FW, FH+2);
  if( fireActive ) display.print("FIRE");
  else display.print("fire");
}

void updateTimeEnv() {
  display.setCursor(0, H-FH+2);
  display.print("day1");

  display.setCursor(H-4*FW, H-FH+2);
  display.print("light");
}

void updateActive() {
  for ( int i = 0; i < 6; i++ ) {
    if ( isLegActive[i] ) display.fillTriangle(legVert[i][0][0], legVert[i][0][1], legVert[i][1][0], legVert[i][1][1], legVert[i][2][0], legVert[i][2][1], WHITE);
    else display.drawTriangle(legVert[i][0][0], legVert[i][0][1], legVert[i][1][0], legVert[i][1][1], legVert[i][2][0], legVert[i][2][1], WHITE);
  }
  for ( int i = 0; i < 3; i++ ) {
    if ( isDistActive[i] ) display.fillTriangle(distVert[i][0][0], distVert[i][0][1], distVert[i][1][0], distVert[i][1][1], distVert[i][2][0], distVert[i][2][1], WHITE);
    else display.drawTriangle(distVert[i][0][0], distVert[i][0][1], distVert[i][1][0], distVert[i][1][1], distVert[i][2][0], distVert[i][2][1], WHITE);
  }
}
void initializeVertices() {
  // legs
  float R = (float)H / 2.0;
  float r = R / 3.0 * sqrt(3.0);

  float gam = 3.0;

  // angles
  float angle = TWO_PI / 6.0;
  float offset = TWO_PI / 6.0 / 2.0;
  float rot = TWO_PI / 6.0 / 2.0;

  for (int i = 0; i < 6; i++) {
    // outer perimeter
    legVert[i][0][0] = (int)(cos(angle * i + rot) * R + R);
    legVert[i][0][1] = (int)(sin(angle * i + rot) * R + R);
    // inner perimeter
    legVert[i][1][0] = (int)(cos(angle * i + offset + rot) * r + R);
    legVert[i][1][1] = (int)(sin(angle * i + offset + rot) * r + R);

    legVert[i][2][0] = (int)(cos(angle * i - offset + rot) * r + R);
    legVert[i][2][1] = (int)(sin(angle * i - offset + rot) * r + R);

    // distance sensors
    if ( (i % 2) == 0 ) {
      distVert[i / 2][0][0] = (int)(cos(angle * i + rot) * gam + R);
      distVert[i / 2][0][1] = (int)(sin(angle * i + rot) * gam + R);

      distVert[i / 2][1][0] = (int)(cos(angle * i + offset + rot) * (r - 3.0) + R);
      distVert[i / 2][1][1] = (int)(sin(angle * i + offset + rot) * (r - 3.0) + R);

      distVert[i / 2][2][0] = (int)(cos(angle * i - offset + rot) * (r - 3.0) + R);
      distVert[i / 2][2][1] = (int)(sin(angle * i - offset + rot) * (r - 3.0) + R);
    }
  }
}

void testdrawchar(void) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  for (uint8_t i = 0; i < 168; i++) {
    if (i == '\n') continue;
    display.write(i);
    if ((i > 0) && (i % 21 == 0))
      display.println();
  }
  display.display();
  delay(1);
}

void testdrawcircle(void) {
  for (int16_t i = 0; i < display.height(); i += 2) {
    display.drawCircle(display.width() / 2, display.height() / 2, i, WHITE);
    display.display();
    delay(1);
  }
}

void testfillrect(void) {
  uint8_t color = 1;
  for (int16_t i = 0; i < display.height() / 2; i += 3) {
    // alternate colors
    display.fillRect(i, i, display.width() - i * 2, display.height() - i * 2, color % 2);
    display.display();
    delay(1);
    color++;
  }
}

void testdrawtriangle(void) {
  for (int16_t i = 0; i < min(display.width(), display.height()) / 2; i += 5) {
    display.drawTriangle(display.width() / 2, display.height() / 2 - i,
                         display.width() / 2 - i, display.height() / 2 + i,
                         display.width() / 2 + i, display.height() / 2 + i, WHITE);
    display.display();
    delay(1);
  }
}

void testfilltriangle(void) {
  uint8_t color = WHITE;
  for (int16_t i = min(display.width(), display.height()) / 2; i > 0; i -= 5) {
    display.fillTriangle(display.width() / 2, display.height() / 2 - i,
                         display.width() / 2 - i, display.height() / 2 + i,
                         display.width() / 2 + i, display.height() / 2 + i, WHITE);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    display.display();
    delay(1);
  }
}

void testdrawroundrect(void) {
  for (int16_t i = 0; i < display.height() / 2 - 2; i += 2) {
    display.drawRoundRect(i, i, display.width() - 2 * i, display.height() - 2 * i, display.height() / 4, WHITE);
    display.display();
    delay(1);
  }
}

void testfillroundrect(void) {
  uint8_t color = WHITE;
  for (int16_t i = 0; i < display.height() / 2 - 2; i += 2) {
    display.fillRoundRect(i, i, display.width() - 2 * i, display.height() - 2 * i, display.height() / 4, color);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    display.display();
    delay(1);
  }
}

void testdrawrect(void) {
  for (int16_t i = 0; i < display.height() / 2; i += 2) {
    display.drawRect(i, i, display.width() - 2 * i, display.height() - 2 * i, WHITE);
    display.display();
    delay(1);
  }
}

void testdrawline() {
  for (int16_t i = 0; i < display.width(); i += 4) {
    display.drawLine(0, 0, i, display.height() - 1, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i = 0; i < display.height(); i += 4) {
    display.drawLine(0, 0, display.width() - 1, i, WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();
  for (int16_t i = 0; i < display.width(); i += 4) {
    display.drawLine(0, display.height() - 1, i, 0, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i = display.height() - 1; i >= 0; i -= 4) {
    display.drawLine(0, display.height() - 1, display.width() - 1, i, WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();
  for (int16_t i = display.width() - 1; i >= 0; i -= 4) {
    display.drawLine(display.width() - 1, display.height() - 1, i, 0, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i = display.height() - 1; i >= 0; i -= 4) {
    display.drawLine(display.width() - 1, display.height() - 1, 0, i, WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();
  for (int16_t i = 0; i < display.height(); i += 4) {
    display.drawLine(display.width() - 1, 0, 0, i, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i = 0; i < display.width(); i += 4) {
    display.drawLine(display.width() - 1, 0, i, display.height() - 1, WHITE);
    display.display();
    delay(1);
  }
  delay(250);
}

void testscrolltext(void) {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.clearDisplay();
  display.println("scroll");
  display.display();
  delay(1);

  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
}
