#include <exadc.h>
#include <i2clite.h>
#include <avr/io.h>
#include <util/delay.h>

#define MCP3426_I2C_WRITE_ADDR 0b11010000
#define MCP3426_I2C_READ_ADDR (MCP3426_I2C_WRITE_ADDR | 1)

void exadc_init()
{
}

uint16_t exadc_read(uint8_t chan)
{
	i2c_repStart(MCP3426_I2C_WRITE_ADDR);
	// start a conversion
	// 1x PGA gain
	// 15 SPS, 16 bit
	// one-shot mode
	i2c_write(0b10001000 | ((chan & 0x03) << 5));
	i2c_stop();

	_delay_ms(70); // 15 SPS means 0.0666 seconds between samples

	i2c_repStart(MCP3426_I2C_READ_ADDR);
	uint16_t r = i2c_readAck() << 8;
	r |= i2c_readNak();

	return r;
}