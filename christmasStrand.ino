#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN 6
#define TIMEOUT 15

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(300, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

char intensity[512];
String inputString = "";
boolean footerReceived = false;
uint8_t idle_counter = 0;

unsigned long t0 = millis();
unsigned long second_counter = millis();

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  // initialize the second counter
  second_counter = millis();

  set_on();

  //Reserve space for the inputString and buffer
  inputString.reserve(512);
  
  Serial.begin(115200);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void set_on() {
  for ( uint16_t i = 0; i < 512; i++ ) {
    intensity[i] =  32;
  }
  
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
}

void serialDelay(uint8_t wait) {
  // do the timery things
  t0 = millis();

  while (((millis() - t0) < wait)) {
    // Add some time stuff
    if ((millis() - second_counter) > 1000) {
      idle_counter += 1;
      second_counter = millis();

      if (idle_counter > TIMEOUT) {
        set_on();
      }
    }

    // Check for new serial data
    serialEvent();
    if (footerReceived) {
      inputString.toCharArray(intensity, 512);
      inputString = "";
      setSimpleLights(intensity[150]);
      footerReceived = false;
      Serial.write(intensity[150]);
      Serial.write(0xdb);
    }
  }
  //Serial.write('A');
}

void serialEvent() {
  while (Serial.available()) {
    // reset the idle timer
    idle_counter = 0;
    // get the new byte:
    char inChar = (char)Serial.read();
    //DEBUG: write it back
    //Serial.write(inChar);
    // add it to the inputString:
    inputString += inChar;
    char blah = 0xDB;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == blah) {
      footerReceived = true;
    }
  }
}

void setSimpleLights(uint8_t inByte) {
    digitalWrite(2, bitRead(inByte, 0));
    digitalWrite(3, bitRead(inByte, 1));
    digitalWrite(4, bitRead(inByte, 2));
    digitalWrite(5, bitRead(inByte, 3));
}

//uint8_t getBrightness(uint8_t b) {
//  uint8_t newBrightness = b + 1;
//  
//    // Brightness has changed -- re-scale existing data in RAM
//    uint8_t  c,
//            *ptr           = pixels,
//             oldBrightness = brightness - 1; // De-wrap old brightness value
//    uint16_t scale;
//    if(oldBrightness == 0) scale = 0; // Avoid /0
//    else if(b == 255) scale = 65535 / oldBrightness;
//    else scale = (((uint16_t)newBrightness << 8) - 1)
//    for(uint16_t i=0; i<numBytes; i++) {
//      c      = *ptr;
//      *ptr++ = (c * scale) >> 8;
//    }
//  
//}


void setPixelColorWithIntensity(uint16_t n, uint32_t c) {
      uint8_t *p,
      r = (uint8_t)(c >> 16),
      g = (uint8_t)(c >>  8),
      b = (uint8_t)c;

      uint8_t brightness = intensity[n % 150];
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;

      strip.setPixelColor(n, r, g, b);
}

void loop() {
  // Some example procedures showing how to display to the pixels:
//  colorWipe(strip.Color(255, 0, 0), 16); // Red
//  colorWipe(strip.Color(0, 255, 0), 16); // Green
//  colorWipe(strip.Color(0, 0, 255), 16); // Blue

  rainbow(16);
  rainbowCycle(16);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    setPixelColorWithIntensity(i, c);
    strip.show();
    serialDelay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      setPixelColorWithIntensity(i, Wheel((i + j) & 255));
    }
    strip.show();
    serialDelay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j = j + 8) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      setPixelColorWithIntensity(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    serialDelay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
