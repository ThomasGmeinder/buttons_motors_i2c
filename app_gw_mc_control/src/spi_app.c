

#include "common.h"

#if ACCESS_ADC_VIA_SPI
extern int adc_values[NUM_ADC_CHANNELS];

int get_adc_value(unsigned adc_chan) {
	return adc_values[adc_chan];
}
#endif