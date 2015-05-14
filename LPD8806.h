// Mbed library to control LPD8806-based RGB LED Strips
// (c) 2011 Jelmer Tiete
// This library is ported from the Arduino implementation of Adafruit Industries
// found at: http://github.com/adafruit/LPD8806
// and their strips: http://www.adafruit.com/products/306
// Released under the MIT License: http://mbed.org/license/mit
//
// standard connected to 1st hardware SPI
// LPD8806  <> MBED
// DATA     -> P5
// CLOCK    -> p7
/*****************************************************************************/

#include "mbed.h"
#ifndef MBED_LPD8806_H
#define MBED_LPD8806_H

class LPD8806 {

 public:

  LPD8806(uint16_t n);
  void
    begin(void),
    show(void),
    setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b),
    setPixelColor(uint16_t n, uint32_t c);
  uint16_t
    numPixels(void);
  uint32_t
    Color(uint8_t, uint8_t, uint8_t);

 private:

  uint8_t
    *pixels;     // Holds LED color values
  uint16_t
    numLEDs;     // Number of RGB LEDs in strand
    
  void
    writezeros(uint16_t n);
};
#endif
