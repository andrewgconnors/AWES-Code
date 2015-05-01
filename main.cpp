#include "mbed.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include "LPD8806.h"

DigitalOut led(p25);

LPD8806 strip = LPD8806(20);

enum modes_t { BPMSlow, BPMFast, STANDBY, CHECKER, OFF } MODE; // current mode of operation
DigitalIn modeButton(p14);

Ticker x;
Ticker t; // ticker used to time sampling
int sampleNumber = 0; // number used to fill the buffer array
int updateFlag = 0; // control when the FFT updates

const uint16_t SEQUENCE_LENGTH = 1024; // number of samples in a frame
const uint16_t OUTPUT_BAND_QUANTITY = 8; // number of output frequency bands
const uint16_t BPM_BAND_QUANTITY = 32; // number of frequency bands for BPM calculation
const uint16_t HISTORY_QUANTITY = 39; // 39 frames of history, so one second's worth of history

AnalogOut musicOut(p18);

DigitalOut m0(p5);
DigitalOut m1(p6);
DigitalOut m2(p7);
DigitalOut m3(p8);
DigitalOut m4(p27);
DigitalOut m5(p28);
DigitalOut m6(p29);
DigitalOut m7(p26);

DigitalOut motors[] = {m0, m1, m2, m3, m4, m5, m6, m7};

// Buffers used to track state of the machine
float32_t audio_buffer[SEQUENCE_LENGTH];
float32_t audio_out_buffer[SEQUENCE_LENGTH];
float32_t fft_in_buffer[SEQUENCE_LENGTH];
float32_t fft_out_buffer[SEQUENCE_LENGTH];

// BPM mode settings
float32_t energy_buffer[BPM_BAND_QUANTITY];
float32_t energy_history_buffer[BPM_BAND_QUANTITY][HISTORY_QUANTITY + 1]; // add one to history quantity to keep an average in the last index
uint16_t energy_history_buffer_position = 0;
int beatDetect = 0;
uint16_t output_bubble_state = 0x01; // 8 bits that set each motor's power
bool direction = true; // true = up, false = down

void columnColor(uint32_t c, uint8_t column) {
    
    if (column == 0) {
        strip.setPixelColor(0, c);
        strip.setPixelColor(1, c);
        strip.setPixelColor(2, c);    
    }
    
    else if (column == 1) {
        strip.setPixelColor(3, c);
        strip.setPixelColor(4, c);
    }
    
    else if (column == 2) {
        strip.setPixelColor(5, c);
        strip.setPixelColor(6, c);
        strip.setPixelColor(7, c);    
    }
    
    else if (column == 3) {
        strip.setPixelColor(8, c);
        strip.setPixelColor(9, c);
    }
    
    else if (column == 4) {
        strip.setPixelColor(10, c);
        strip.setPixelColor(11, c);
        strip.setPixelColor(12, c);    
    }
    
    else if (column == 5) {
        strip.setPixelColor(13, c);
        strip.setPixelColor(14, c);
    }
    
    else if (column == 6) {
        strip.setPixelColor(15, c);
        strip.setPixelColor(16, c);
        strip.setPixelColor(17, c);    
    }
    
    else if (column == 7) {
        strip.setPixelColor(18, c);
        strip.setPixelColor(19, c);
    }
    
    strip.show();
}

arm_cfft_instance_f32 CFFT = {SEQUENCE_LENGTH/2, twiddleCoef_512, armBitRevIndexTable512, ARMBITREVINDEXTABLE_512_TABLE_LENGTH}; // initialize complex FFT
arm_rfft_fast_instance_f32 RFFT = {CFFT, SEQUENCE_LENGTH, (float32_t*)twiddleCoef_rfft_1024}; // initialize real FFT   

// Read the analog pin 1024 times, then set an update flag to analyze frame
void sampleOverWindow() {
    musicOut = audio_out_buffer[sampleNumber];
    while((LPC_ADC->ADSTAT & 0x2) == 0);
    audio_buffer[sampleNumber] = (float32_t)(((float32_t)((LPC_ADC->ADDR1 >> 4) & 0xFFF))/((float32_t)0xFFF)); // read input voltage level
    sampleNumber++;
    if(sampleNumber == SEQUENCE_LENGTH) {
        updateFlag = 1;
        sampleNumber = 0;
    }
}

void subtractMean(float32_t * array, uint16_t arraySize) {
    float32_t sum = 0;
    for(uint16_t i = 0; i < arraySize; i++) {
        sum += array[i];
    }
    sum = sum/arraySize;
    for(uint16_t i = 0; i < arraySize; i++) {
        array[i] -= sum;
    }
}

float32_t arraySum(uint16_t startIndex, uint16_t stopIndex, float32_t *array) {
    float32_t sum = 0;
    for(uint16_t i = startIndex; i < stopIndex; i++) {
        sum += fabs(array[i]);
    }
    return sum;
}

void updateBubbles() {
    for(int i = 0; i < OUTPUT_BAND_QUANTITY; i++) {
        unsigned int level = (output_bubble_state >> i) & 1;
        motors[i].write(level);
    }
}

void BPMLights() {
    for(int i = 0; i < 8; i++) {
        //columnColor(motors[i].read()*strip.Color(127*((i+1)&4), 127*((i+1)&2), 127*((i+1)&1)), i);
        uint8_t r = (127*((i+1)>>2))*(output_bubble_state >> i & 1);
        uint8_t g = 127*(((i+1)>>1)&1)*(output_bubble_state >> i & 1);
        uint8_t b = 127*((i+1)&1)*(output_bubble_state >> i & 1);
        columnColor(strip.Color(r, g, b), i);
    }
}

void StandByLights() {
    columnColor(strip.Color(127*(output_bubble_state & 1), 0, 0), 0);
    columnColor(strip.Color(127*(output_bubble_state >> 7 & 1), 0, 0), 7);
    
    columnColor(strip.Color(0, 0, 127*(output_bubble_state >> 1 & 1)), 1);
    columnColor(strip.Color(0, 0, 127*(output_bubble_state >> 6 & 1)), 6);
    
    columnColor(strip.Color(0, 127*(output_bubble_state >> 2 & 1), 0), 2);
    columnColor(strip.Color(0, 127*(output_bubble_state >> 5 & 1), 0), 5);
    
    columnColor(strip.Color(127*(output_bubble_state >> 3 & 1), 0, 127*(output_bubble_state >> 3 & 1)), 3);
    columnColor(strip.Color(127*(output_bubble_state >> 4 & 1), 0, 127*(output_bubble_state >> 4 & 1)), 4);
    
}

void checkerBubble() {
    output_bubble_state = ~output_bubble_state;
    updateBubbles();
}

uint32_t Wheel(uint16_t WheelPos) {
    uint8_t b=0;
    uint8_t g=0;
    uint8_t r = 0;
    switch (WheelPos / 128) {
        case 0:
            r = 127 - WheelPos % 128;   //Red down
            g = WheelPos % 128;      // Green up
            b = 0;                  //blue off
            break;
        case 1:
            g = 127 - WheelPos % 128;  //green down
            b = WheelPos % 128;      //blue up
            r = 0;                  //red off
            break;
        case 2:
            b = 127 - WheelPos % 128;  //blue down
            r = WheelPos % 128;      //red up
            g = 0;                  //green off
            break;
    }
    return(strip.Color(r,g,b));
}

void rainbowCycle(uint8_t delay) {
    uint16_t i, j;

    for (j=0; j < 384 * 5; j++) {     // 5 cycles of all 384 colors in the wheel
        for (i=0; i < strip.numPixels(); i++) {
            // tricky math! we use each pixel as a fraction of the full 384-color wheel
            // (thats the i / strip.numPixels() part)
            // Then add in j which makes the colors go around per pixel
            // the % 384 is to make the wheel cycle around
            strip.setPixelColor(i, Wheel( ((i * 384 / strip.numPixels()) + j) % 384) );
        }
        strip.show();   // write all the pixels out
        wait_ms(delay);
    }
}

void updateState(float32_t *fft_current_output, uint16_t bands, float ratio) {
    // Update BPM buffers then update frequency spectrum buffers
    uint16_t scale = SEQUENCE_LENGTH/BPM_BAND_QUANTITY;
    fft_current_output[0] = 0;
    
    for(uint16_t i = 0; i < bands; i++) { // analyze <625 Hz
        energy_buffer[i] = arraySum((scale*i), (scale*(i+1)), fft_current_output)/BPM_BAND_QUANTITY;
        energy_history_buffer[i][HISTORY_QUANTITY] = arraySum(0, HISTORY_QUANTITY, energy_history_buffer[i])/HISTORY_QUANTITY;
        energy_history_buffer[i][energy_history_buffer_position] = energy_buffer[i];
        if(energy_history_buffer_position == HISTORY_QUANTITY - 1) energy_history_buffer_position = 0;
        else energy_history_buffer_position++;
        if(energy_buffer[i] > ratio*energy_history_buffer[i][HISTORY_QUANTITY]) beatDetect++;
    }
}

void switchMode() {
    led.write(0);
    switch(MODE) {
        case BPMSlow:
            output_bubble_state = 0x01;
            MODE = BPMFast;
            break;
        case BPMFast:
            output_bubble_state = 0x81;
            musicOut.write(0.0);
            MODE = STANDBY;
            break;
        case STANDBY:
            output_bubble_state = 0x55;
            musicOut.write(0.0);
            MODE = CHECKER;
            break;
        case CHECKER:
            x.detach();
            output_bubble_state = 0x00;
            musicOut.write(0.0);
            MODE = OFF;
            updateBubbles();
            for (int i=0; i < strip.numPixels(); i++) {
                strip.setPixelColor(i, 0);  // turn all pixels off
            }
            strip.show();
            break;
        case OFF:
            output_bubble_state = 0x01;
            MODE = BPMSlow;
            break;
    }
    while(modeButton.read() == 1);
    led.write(1);
}

int main() {
    
    MODE = BPMSlow;
    led.write(1);
    
    strip.begin();
    strip.show();
    
    for (int i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, 0);  // turn all pixels off
    }
    strip.show();
    
    //checkerColor(strip.Color(0, 127, 127), 50);

    // ADC Configuration
    LPC_SC->PCONP |= 0x00001000; // enable ADC power
    LPC_SC->PCLKSEL0 |= 0x03000000; // select CCLK/8 for the ADC, so 96/8 = 12 MHz
    LPC_PINCON->PINSEL1 |= 0x00010000; // set pin 0.24 (p16) to AD0.1 mode
    LPC_PINCON->PINMODE1 |= 0x00020000; // set neither pull-up nor pull-down resistor mode on pin 0.24
    //LPC_PINCON->PINMODE0 |= 0x000000F0; // set pull-down resistors on pins 0.02, 0.03
    //LPC_PINCON->PINMODE1 |= 0x003EC000; // set neither pull-up nor pull-down resistor mode on pin 0.24 and pull-down on pins 0.23, 0.25, 0.26
    //LPC_PINCON->PINMODE3 |= 0xF0000000; // set pull-down resistors on pins 1.30, 1.31
    LPC_ADC->ADCR |= 0x00210002; // set ADC to be operational, set SEL to AD0.1, CLKDIV to 0, BURST to 1, and START to 000
    
    t.attach_us(&sampleOverWindow, 25); // read the analog input every 25 us
    
    while(1) {
        if(modeButton.read() == 1) switchMode();
        switch(MODE)
        {
        case BPMSlow:
            if(updateFlag) {
                updateFlag = 0;
                memcpy(fft_in_buffer, audio_buffer, sizeof audio_buffer);
                memcpy(audio_out_buffer, fft_in_buffer, sizeof audio_buffer);
                subtractMean(fft_in_buffer, SEQUENCE_LENGTH);
                arm_rfft_fast_f32(&RFFT, fft_in_buffer, fft_out_buffer, 0); // update FFT
                updateState(fft_out_buffer, 1, 1.25);
            }
            if(beatDetect > 3) {
                if(direction) output_bubble_state <<= 1;
                else output_bubble_state >>= 1;
                if(output_bubble_state == 1) direction = true;
                else if(output_bubble_state == 0x80) direction = false;
                else if(output_bubble_state == 0x00) output_bubble_state = 0x01;
                updateBubbles();
                BPMLights();
                //printf("%u\n", output_bubble_state);
                beatDetect = 0;
            }
            break;
        case BPMFast:
            if(updateFlag) {
                updateFlag = 0;
                memcpy(fft_in_buffer, audio_buffer, sizeof audio_buffer);
                memcpy(audio_out_buffer, fft_in_buffer, sizeof audio_buffer);
                subtractMean(fft_in_buffer, SEQUENCE_LENGTH);
                arm_rfft_fast_f32(&RFFT, fft_in_buffer, fft_out_buffer, 0); // update FFT
                updateState(fft_out_buffer, 31, 1.4);
            }
            if(beatDetect > 3) {
                if(direction) output_bubble_state <<= 1;
                else output_bubble_state >>= 1;
                if(output_bubble_state == 1) direction = true;
                else if(output_bubble_state == 0x80) direction = false;
                else if(output_bubble_state == 0x00) output_bubble_state = 0x01;
                updateBubbles();
                BPMLights();
                //printf("%u\n", output_bubble_state);
                beatDetect = 0;
            }
            break;
        case STANDBY:
            if(direction) output_bubble_state = ((output_bubble_state & 0xF0) << 1) + ((output_bubble_state & 0x0F) >> 1); // move out
            else output_bubble_state = ((output_bubble_state & 0xF0) >> 1) + ((output_bubble_state & 0x0F) << 1); // move in
            if(output_bubble_state == 0x18) direction = true;
            else if(output_bubble_state == 0x81) direction = false;
            else if(output_bubble_state == 0x00) {
                output_bubble_state = 0x81;
                direction = false;
            }
            updateBubbles();
            StandByLights();
            wait_ms(500);
            break;
        case CHECKER:
            x.attach(&checkerBubble, .5);
            rainbowCycle(5);
            break;
        case OFF:
            break;
        }
    }
}
