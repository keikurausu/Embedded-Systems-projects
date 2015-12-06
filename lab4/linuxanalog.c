#include "linuxanalog.h"
#include <inttypes.h>


void das1602_initialize()
{
	uint16_t BADR_config;
	BADR_config = 0;
	BADR_config = LDAEMCL | DACEN | HS0 | START;
	//set BADR1+8 configuration parameters
	outw(BADR_config, BADR1 + 8);
	//clear the DAC FIFO buffer
	outw(0, BADR4 + 2);
}

void dac(uint16_t value)
{
	outw(value, BADR4);
}
