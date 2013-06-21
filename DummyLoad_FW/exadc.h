#ifndef exadc_h
#define exadc_h

#include <stdint.h>

void exadc_init();
uint16_t exadc_read(uint8_t);

#endif