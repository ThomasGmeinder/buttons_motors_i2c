// Copyright (c) 2016, XMOS Ltd, All rights reserved
#include <xs1.h>
#include <spi.h>
#include <syscall.h>
#include <timer.h>
#include <print.h>
#include <platform.h>
#include <stdio.h>
#include "common.h"
#include "dsp.h"
#include "xscope.h"

int read_mpc3008_adc_channel(client spi_master_if spi, unsigned adc_chan, unsigned spi_clk_khz) {

    uint8_t val;
    int adc_val = 0;

    // 0x80 is single ended bit
    uint8_t command = 0x80 | (adc_chan & 0x7) << 4;

    //printf("Reading ADC channel %d with command 0x%x\n", adc_chan, command);

    spi.begin_transaction(0, spi_clk_khz, SPI_MODE_2);
    val = spi.transfer8(0x1); // Start bit

    val = spi.transfer8(command); // Single, D2..D0
    val &= 0x3; // mask off the undefined bits
    adc_val = (val << 8);
    val = spi.transfer8(0x0);  // Read remaining bits
    adc_val |= val;
    spi.end_transaction(spi_clk_khz);

    return adc_val;
}

/* This application function sends some traffic as SPI master using
 * the synchronous interface. Since this is run in simulation
 * there is no slave, so the incoming data (stored in the 'val'
 * variable) will just be zero.
 */

#if ACCESS_ADC_VIA_SPI

#define SAMPLES_PER_SINE 300
#define ADC_FS (50 * SAMPLES_PER_SINE) // ADC Sample Frequency for 50 Hz sine
#define ADC_SAMPLING_INTERVAL XS1_TIMER_HZ/ADC_FS

#if ADC_SAMPLING_INTERVAL < 6600
// Measured 6526 cycles to input and process ADC values from two channels
#error "ADC_SAMPLING_INTERVAL is too small"
#endif

#define ADC_MIN 0x0
#define ADC_MAX 0x3ff
#define ADC_HALF ADC_MAX>>1

#define SENSITIVITY Q16(10) // 10 A/V (20A module is 100mV/A) 

unsigned adc_value[NUM_ADC_CHANNELS];
int32_t current_rms_q16[NUM_ADC_CHANNELS];

unsigned average_counter[NUM_ADC_CHANNELS];

unsigned min_adc_val[NUM_ADC_CHANNELS] = {ADC_MAX, ADC_MAX};  
unsigned max_adc_val[NUM_ADC_CHANNELS] = {ADC_MIN, ADC_MIN};  
// Channel 0 has an issue with the ADC used on the second system
// So this is a SW fix for a HW issue.
unsigned used_adc_chans[NUM_ADC_CHANNELS] = {1, 2};

#define GEN_ADC_VALUE 0

int32_t adc_sine_values[SAMPLES_PER_SINE];

void process_adc_value(unsigned c, int adc_value) {

  if(adc_value < min_adc_val[c]) min_adc_val[c] = adc_value;
  if(adc_value > max_adc_val[c]) max_adc_val[c] = adc_value;

  average_counter[c]++;

  if(average_counter[c] >= SAMPLES_PER_SINE) {
    int32_t delta = max_adc_val[c] - min_adc_val[c];

    // peak to peak voltage
    int32_t Vpp = dsp_math_multiply(Q16(delta), Q16(5), 16);
    Vpp = dsp_math_divide(Vpp, Q16(ADC_MAX), 16); // Actual voltage
    int32_t Vp = Vpp/2; // Peak voltage
    int32_t Vrms = dsp_math_multiply(Vp, Q16(0.707106781186548), 16); // multiply with sqrt(2)
    current_rms_q16[c] = dsp_math_multiply(Vrms, SENSITIVITY, 16);
    
    // reset values
    average_counter[c]=0;
    min_adc_val[c]=ADC_MAX;  
    max_adc_val[c]=ADC_MIN; 
  }
  // xscope_float doesn't display correctly
  if(c==0) xscope_int(CURRENT_RMS, current_rms_q16[c]);
}


#if GEN_ADC_VALUE
void pregen_adc_values() {
  int32_t adc_value;
  for(unsigned i=0; i<SAMPLES_PER_SINE; ++i) {
    int64_t rad = (int64_t) i*PI2_Q8_24/SAMPLES_PER_SINE;
    q8_24 sine = dsp_math_sin(rad); 
    printf("Generated sine value: %.2f\n", F24(sine));
    // continue with Q16
    sine = sine >> 8;
    adc_value = dsp_math_multiply(sine, Q16(ADC_HALF), 16) + Q16(ADC_HALF);

    //adc_value = (sine >> 8) * ADC_HALF + Q16(ADC_HALF);
    adc_value = adc_value >> 16; // truncate fractional
    printf("Generated ADC value: 0x%x\n", adc_value);
    adc_sine_values[i] = adc_value;
  }
}
#endif


void spi_app(client spi_master_if spi)
{

    timer sample_tmr;
    int t;

#if MEASURE_TIME
    timer measure_tmr;
    int t0, t1;
#endif 

    const unsigned spi_clk_khz = 1000;
    printstrln("Starting SPI access");
    delay_microseconds(100);

    printf("Reading all available ADC channels for testing:\n");
    for(unsigned c=0; c<8; ++c) {
        printf("ADC %d: 0x%x\n", c, read_mpc3008_adc_channel(spi, c, spi_clk_khz));
    }

    sample_tmr :>  t;

    printf("Start processing ADC inputs with a period of %d ref clock cycles\n", ADC_SAMPLING_INTERVAL);

#if GEN_ADC_VALUE
    pregen_adc_values();
#endif

    while(1) {
        sample_tmr when timerafter(t+ADC_SAMPLING_INTERVAL) :> t; 
#if MEASURE_TIME
        measure_tmr :> t0;
#endif
        for(unsigned i=0; i<NUM_ADC_CHANNELS; ++i) {
            unsigned c = used_adc_chans[i];
            adc_value[i] = read_mpc3008_adc_channel(spi, c, spi_clk_khz);
#if GEN_ADC_VALUE
            adc_value[i] = adc_sine_values[average_counter[i]];
            adc_value[i] >>= c; // shift by channel index to distinguish channels
#endif
            process_adc_value(i, adc_value[i]);
        }
#if MEASURE_TIME
        measure_tmr :> t1;
        printf("Sampling and Processing ADC values from %d channels takes %d cycles\n", NUM_ADC_CHANNELS, t1-t0);
#endif
        xscope_int(ADC, adc_value[0]);

    }


}
#endif
