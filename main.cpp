#include "mbed.h"
#include "arm_math.h"
#include "arm_common_tables.h"

Ticker t; // ticker used to time sampling
int sampleNumber = 0; // number used to fill the buffer array
int updateFlag = 0; // control when the FFT updates

const uint16_t SEQUENCE_LENGTH = 1024;
const uint16_t BAND_QUANTITY = 8;

float32_t audio_buffer[SEQUENCE_LENGTH];
float32_t fft_in_buffer[SEQUENCE_LENGTH];
float32_t fft_out_buffer[SEQUENCE_LENGTH];
float32_t output_state[BAND_QUANTITY];

arm_cfft_instance_f32 CFFT = {SEQUENCE_LENGTH, twiddleCoef_1024, armBitRevIndexTable1024, ARMBITREVINDEXTABLE1024_TABLE_LENGTH}; // initialize complex FFT
arm_rfft_fast_instance_f32 RFFT = {CFFT, SEQUENCE_LENGTH, (float32_t*)twiddleCoef_rfft_1024}; // initialize real FFT   

void sampleOverWindow() {
    audio_buffer[sampleNumber] = (float32_t)(((float32_t)((LPC_ADC->ADGDR >> 4) & 0xFFF))/((float32_t)0xFFF)); // read input voltage level
    sampleNumber++;
    if(sampleNumber == 1024) {
        updateFlag = 1;
        sampleNumber = 0;
    }
}

float32_t arraySumSquared(uint16_t startIndex, uint16_t stopIndex, float32_t *array) {
    float32_t sum = 0;
    for(uint16_t i = startIndex; i < stopIndex; i++) {
        sum += array[i]*array[i];
    }
    return sum;
}

void updateOutput(float32_t *fft_current_output) {
    uint16_t scale = SEQUENCE_LENGTH/(2*BAND_QUANTITY);
    printf("Scale = %u\n", scale);
    fft_current_output[0] = 0;
    for(uint16_t i = 0; i < BAND_QUANTITY; i++) {
        output_state[i] = arraySumSquared(scale*i, scale*(i+1), fft_current_output);
    }
}

int main() {
    
    // ADC Configuration
    LPC_SC->PCONP |= 0x00001000; // enable ADC power
    LPC_SC->PCLKSEL0 |= 0x03000000; // select CCLK/8 for the ADC, so 96/8 = 12 MHz
    LPC_PINCON->PINSEL1 |= 0x00100000; // set pin 0.26 (p18) to AD0.3 mode
    LPC_PINCON->PINMODE1 |= 0x00200000; // set neither pull-up nor pull-down resistor mode
    LPC_ADC->ADCR |= 0x00210008; // set ADC to be operational, set SEL to AD0.3, CLKDIV to 0, BURST to 1, and START to 000
    
    t.attach_us(&sampleOverWindow, 25); // read the analog input every 25 us
    
    while(1) {
        if(updateFlag) {
            memcpy(fft_in_buffer, audio_buffer, sizeof audio_buffer);
            arm_rfft_fast_f32(&RFFT, fft_in_buffer, fft_out_buffer, 0); // update FFT
            updateFlag = 0;
            updateOutput(fft_out_buffer);
            for(int i = 0; i < BAND_QUANTITY; i++) {
                printf("Current Output %d: %f\n", i, output_state[i]);
            }
            //t.attach_us(&sampleOverWindow, 25);
        }
        
    }
}
