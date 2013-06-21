#include <i2clite.h>
#include <avr/io.h>
#include <util/twi.h>

uint8_t i2c_errorFlag = 0;

void i2c_init() {
	// pins to input with external pull-up resistors
	DDRD &= ~(_BV(0) | _BV(1));
	PORTD &= ~(_BV(0) | _BV(1));
	
	TWSR = 0; // no prescaler => prescaler = 1
	TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate
	TWCR = _BV(TWEN); // enable twi module, no interrupt
	
	i2c_errorFlag = 0;
}

void i2c_repStart(uint8_t address) {
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) ; // send REPEAT START condition
	i2c_waitTx(); // wait until transmission completed
	TWDR = address; // send device address
	TWCR = _BV(TWINT) | _BV(TWEN);
	i2c_waitTx(); // wail until transmission completed
}

void i2c_stop() {
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
	loop_until_bit_is_clear(TWCR, TWSTO);
}

void i2c_write(uint8_t data) {
	TWDR = data; // send data to the previously addressed device
	TWCR = _BV(TWINT) | _BV(TWEN);
	i2c_waitTx();
}

uint8_t i2c_readAck() {
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	i2c_waitTx();
	return TWDR;
}

uint8_t i2c_readNak() {
	TWCR = _BV(TWINT) | _BV(TWEN);
	i2c_waitTx();
	uint8_t r = TWDR;
	i2c_stop();
	return r;
}

void i2c_waitTx() {
	uint16_t count = 255;
	while (bit_is_clear(TWCR, TWINT)) {
		count--;
		if (count==0) { //we are in a blocking state => we don't insist
			TWCR = 0; //and we force a reset on TWINT register
			i2c_errorFlag = 1;
			break;
		}
	}
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
	i2c_repStart(add | TW_WRITE);// I2C write direction
	i2c_write(reg); // register selection
	i2c_write(val); // value to write in register
	i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
	i2c_repStart(add | TW_READ); // I2C read direction
	i2c_write(reg); // register selection
	i2c_repStart(add | 1); // I2C read direction
	return i2c_readNak(); // Read single register and return value
}

void i2c_writeBlock(uint8_t add, uint8_t* data, uint8_t len) {
	i2c_repStart(add | TW_WRITE);
	uint8_t i;
	for (i = 0; i < len; i++) {
		i2c_write(data[i]);
	}
	i2c_stop();
}

void i2c_readBlock(uint8_t add, uint8_t* data, uint8_t len) {
	i2c_repStart(add | TW_READ);
	uint8_t i;
	for (i = 0; i < len - 1; i++) {
		data[i] = i2c_readAck();
	}
	data[len - 1] = i2c_readNak();
}