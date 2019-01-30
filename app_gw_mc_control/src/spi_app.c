

#include "common.h"
#include "dsp.h"

#if ACCESS_ADC_VIA_SPI
extern q8_24 current_rms_q16[NUM_ADC_CHANNELS];

int get_rms_current(unsigned adc_chan) {
	return current_rms_q16[adc_chan];
}
#endif