#ifndef _spi_access_h_
#define _spi_access_h_

#include <spi.h>
#include "common.h"

void spi_app(client spi_master_if spi);

int get_adc_value(unsigned adc_chan);

#endif

