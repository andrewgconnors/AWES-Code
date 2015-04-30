#include "mbed.h"
#include "arm_math.h"
#include "arm_common_tables.h"

LocalFileSystem local("local");
FILE *fp = fopen("/local/energies.txt", "w");
DigitalOut led(p25);

enum modes_t { BPM, STANDBY, OFF } MODE; // current mode of operation
DigitalIn modeButton(p15);
int run = 1;

Ticker t; // ticker used to time sampling
int sampleNumber = 0; // number used to fill the buffer array
int updateFlag = 0; // control when the FFT updates

const uint16_t SEQUENCE_LENGTH = 1024; // number of samples in a frame
const uint16_t OUTPUT_BAND_QUANTITY = 8; // number of output frequency bands
const uint16_t BPM_BAND_QUANTITY = 32; // number of frequency bands for BPM calculation
const uint16_t HISTORY_QUANTITY = 39; // 39 frames of history, so one second's worth of history

DigitalOut m0(p5);
DigitalOut m1(p6);
DigitalOut m2(p7);
DigitalOut m3(p8);
DigitalOut m4(p27);
DigitalOut m5(p28);
DigitalOut m6(p29);
DigitalOut m7(p30);

DigitalOut motors[] = {m0, m1, m2, m3, m4, m5, m6, m7};

// Buffers used to track state of the machine
float32_t audio_buffer[SEQUENCE_LENGTH];
float32_t fft_in_buffer[SEQUENCE_LENGTH];
float32_t fft_out_buffer[SEQUENCE_LENGTH];

// BPM mode settings
float32_t energy_buffer[BPM_BAND_QUANTITY];
float32_t energy_history_buffer[BPM_BAND_QUANTITY][HISTORY_QUANTITY + 1]; // add one to history quantity to keep an average in the last index
uint16_t energy_history_buffer_position = 0;
int beatDetect = 0;
uint16_t output_bubble_state = 0x01; // 8 bits that set each motor's power
bool direction = true; // true = up, false = down

arm_cfft_instance_f32 CFFT = {SEQUENCE_LENGTH/2, twiddleCoef_512, armBitRevIndexTable512, ARMBITREVINDEXTABLE_512_TABLE_LENGTH}; // initialize complex FFT
arm_rfft_fast_instance_f32 RFFT = {CFFT, SEQUENCE_LENGTH, (float32_t*)twiddleCoef_rfft_1024}; // initialize real FFT   

// Read the analog pin 1024 times, then set an update flag to analyze frame
void sampleOverWindow() {
    audio_buffer[sampleNumber] = (float32_t)(((float32_t)((LPC_ADC->ADGDR >> 4) & 0xFFF))/((float32_t)0xFFF)); // read input voltage level
    sampleNumber++;
    if(sampleNumber == SEQUENCE_LENGTH) {
        //printf("%f", audio_buffer[1023]);
        updateFlag = 1;
        sampleNumber = 0;
    }
}

void switchMode() {
    fprintf(fp, "button hit");
    run = 0;
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
    //printf("sum = %f\n", sum);
    return sum;
}

void updateBubbles() {
    for(int i = 0; i < OUTPUT_BAND_QUANTITY; i++) {
        unsigned int level = (output_bubble_state >> i) & 1;
        motors[i].write(level);
    }
}

void updateState(float32_t *fft_current_output) {
    // Update BPM buffers then update frequency spectrum buffers
    uint16_t scale = SEQUENCE_LENGTH/BPM_BAND_QUANTITY;
    fft_current_output[0] = 0;
    
    for(uint16_t i = 0; i < 7; i++) { // analyze <200 Hz
        //printf("current e on band: ");
        energy_buffer[i] = arraySum((scale*i), (scale*(i+1)), fft_current_output)/BPM_BAND_QUANTITY;
        //printf("current e average: ");
        energy_history_buffer[i][HISTORY_QUANTITY] = arraySum(0, HISTORY_QUANTITY, energy_history_buffer[i])/39;
        energy_history_buffer[i][energy_history_buffer_position] = energy_buffer[i];
        //printf("newest e history entry: %f\n", energy_history_buffer[i][energy_history_buffer_position]);
        //printf("intended newest e history entry: %f\n", energy_buffer[i]);
        if(energy_history_buffer_position == HISTORY_QUANTITY - 1) energy_history_buffer_position = 0;
        else energy_history_buffer_position++;
        fprintf(fp, "band %u: energy: %f; average: %f\n\r", i, energy_buffer[i], energy_history_buffer[i][HISTORY_QUANTITY]);
        if(energy_buffer[i] > 1.4*energy_history_buffer[i][HISTORY_QUANTITY]) beatDetect++;
        printf("current e: %f; historical e: %f\n", energy_buffer[i], energy_history_buffer[i][HISTORY_QUANTITY]);
    }
}

int main() {
    
    led.write(1);
    MODE = BPM;
    
    // ADC Configuration
    LPC_SC->PCONP |= 0x00001000; // enable ADC power
    LPC_SC->PCLKSEL0 |= 0x03000000; // select CCLK/8 for the ADC, so 96/8 = 12 MHz
    LPC_PINCON->PINSEL1 |= 0x00100000; // set pin 0.26 (p18) to AD0.3 mode
    LPC_PINCON->PINMODE1 |= 0x00200000; // set neither pull-up nor pull-down resistor mode
    LPC_ADC->ADCR |= 0x00210008; // set ADC to be operational, set SEL to AD0.3, CLKDIV to 0, BURST to 1, and START to 000
    
    t.attach_us(&sampleOverWindow, 25); // read the analog input every 25 us
    
    while(run) {
        if(modeButton.read() == 1) switchMode();
        if(updateFlag) {
            updateFlag = 0;
            memcpy(fft_in_buffer, audio_buffer, sizeof audio_buffer);
            subtractMean(fft_in_buffer, SEQUENCE_LENGTH);
            arm_rfft_fast_f32(&RFFT, fft_in_buffer, fft_out_buffer, 0); // update FFT
            updateState(fft_out_buffer);
        }
        if(beatDetect > 0) {
            //printf("beat detected:");
            if(direction) output_bubble_state <<= 1;
            else output_bubble_state >>= 1;
            if(output_bubble_state == 1) direction = true;
            else if(output_bubble_state == 0x80) direction = false;
            else if(output_bubble_state == 0x00) output_bubble_state = 0x01;
            updateBubbles();
            //printf("%u\n", output_bubble_state);
            beatDetect = 0;
        }
    }
    output_bubble_state = 0;
    updateBubbles();
    fclose(fp);
    led.write(0);
}
