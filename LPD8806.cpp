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

#include "LPD8806.h"

//Define SPI pins
//Connected to first SPI module
//SPI spi(p5, NC, p7); // mosi, miso, sclk
SPI spi(p11, NC, p13); // mosi, miso, sclk

LPD8806::LPD8806(uint16_t n) {
    // Allocate 3 bytes per pixel:
    if (NULL != (pixels = (uint8_t *)malloc(numLEDs * 3))) {
        memset(pixels, 0x80, numLEDs * 3); // Init to RGB 'off' state
        numLEDs     = n;
    }
}

void LPD8806::begin(void) {

    // Setup the spi for 8 bit data, low steady state clock,
    // first edge capture, with a 2MHz clock rate
    spi.format(8,0);
    spi.frequency(2000000);

    // Issue initial latch to 'wake up' strip (latch length varies w/numLEDs)
    writezeros(3 * ((numLEDs + 63) / 64));
}

uint16_t LPD8806::numPixels(void) {
    return numLEDs;
}

void LPD8806::writezeros(uint16_t n) {
    while (n--) spi.write(0x00);
}

// This is how data is pushed to the strip.  Unfortunately, the company
// that makes the chip didnt release the  protocol document or you need
// to sign an NDA or something stupid like that, but we reverse engineered
// this from a strip controller and it seems to work very nicely!
void LPD8806::show(void) {
    uint16_t i, nl3 = numLEDs * 3; // 3 bytes per LED

    for (i=0; i<nl3; i++ ) {
        spi.write(pixels[i]);
    }

    // Write latch at end of data; latch length varies with number of LEDs
    writezeros(3 * ((numLEDs + 63) / 64));

    // We need to have a delay here, a few ms seems to do the job
    // shorter may be OK as well - need to experiment :(
    //wait_ms(3);
}

// Convert R,G,B to combined 32-bit color
uint32_t LPD8806::Color(uint8_t r, uint8_t g, uint8_t b) {
    // Take the lowest 7 bits of each value and append them end to end
    // We have the top bit set high (its a 'parity-like' bit in the protocol
    // and must be set!)
    return 0x808080 | ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
}

// store the rgb component in our array
void LPD8806::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
    if (n >= numLEDs) return; // '>=' because arrays are 0-indexed

    pixels[n*3  ] = g | 0x80;
    pixels[n*3+1] = r | 0x80;
    pixels[n*3+2] = b | 0x80;
}

void LPD8806::setPixelColor(uint16_t n, uint32_t c) {
    if (n >= numLEDs) return; // '>=' because arrays are 0-indexed

    pixels[n*3  ] = (c >> 16) | 0x80;
    pixels[n*3+1] = (c >>  8) | 0x80;
    pixels[n*3+2] =  c        | 0x80;
}
