#ifndef dac_h
#define dac_h

#include <stdint.h>

void dac_init();
void dac_write(uint16_t);

extern uint16_t dac_last;

#endif