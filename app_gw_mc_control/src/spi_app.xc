// Copyright (c) 2016, XMOS Ltd, All rights reserved
#include <xs1.h>
#include <spi.h>
#include <syscall.h>
#include <timer.h>
#include <print.h>
#include <platform.h>
#include <stdio.h>
#include "common.h"

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
int adc_values[NUM_ADC_CHANNELS];

void spi_app(client spi_master_if spi)
{

    const unsigned spi_clk_khz = 1000;
    printstrln("Starting SPI access");
    delay_microseconds(100);

    printf("Reading all available ADC channels for testing:\n");
    for(unsigned c=0; c<8; ++c) {
        printf("ADC %d: 0x%x\n", c, read_mpc3008_adc_channel(spi, c, spi_clk_khz));
    }

    while(1) {
        for(unsigned c=0; c<NUM_ADC_CHANNELS; ++c) {
            adc_values[c] = read_mpc3008_adc_channel(spi, c, spi_clk_khz);
        }
    }
#if 0
    for(unsigned c=0; c<8; ++c) {
        int adc_val = read_mpc3008_adc_channel(spi, c);
        printf("ADC channel %d value: 0x%x\n", c, adc_val);
    }
#endif

}
#endif
