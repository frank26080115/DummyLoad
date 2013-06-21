#include <dac.h>
#include <i2clite.h>
#include <avr/io.h>

#define MAX5812_I2C_WRITE_ADDR 0b00100100
#define MAX5812_I2C_READ_ADDR (MAX5812_I2C_WRITE_ADDR | 1)

uint16_t dac_last;

void dac_init()
{
	i2c_repStart(MAX5812_I2C_WRITE_ADDR);
	i2c_write(0b01000000); // power up
	i2c_stop();
	
	dac_write(0); // default to 0 for safety
}

void dac_write(uint16_t x)
{
	dac_last = x;
	i2c_repStart(MAX5812_I2C_WRITE_ADDR);
	i2c_write(0xC0 | ((x & 0x0F00) >> 8)); // immediate write new DAC data
	i2c_write(x & 0xFF);
	i2c_stop();
}