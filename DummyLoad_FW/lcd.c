#include <lcd.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

// pin defines
#define LCD_CTRL_PORTx		PORTD
#define LCD_CTRL_DDRx		DDRD
#define LCD_CTRL_PINNUM_E	7
#define LCD_CTRL_PINNUM_RS	6
#define _NOP()				asm volatile ("nop")

// tracks current cursor location
uint8_t lcd_curX;
uint8_t lcd_curY;

// send a command to the LCD
static inline void lcd_sendNybble(uint8_t data)
{
	// setup the data bits (last 4 bits)
	if (bit_is_set(data, 3)) PORTC |= _BV(6); else PORTC &= ~_BV(6);
	if (bit_is_set(data, 2)) PORTB |= _BV(6); else PORTB &= ~_BV(6);
	if (bit_is_set(data, 1)) PORTB |= _BV(5); else PORTB &= ~_BV(5);
	if (bit_is_set(data, 0)) PORTB |= _BV(4); else PORTB &= ~_BV(4);
	_delay_us(1);

	// raise the E pin
	LCD_CTRL_PORTx |= _BV(LCD_CTRL_PINNUM_E);
	_delay_us(1);

	// lower the E pin, therefor inputting the data
	LCD_CTRL_PORTx &= ~_BV(LCD_CTRL_PINNUM_E);
	_delay_us(1);
}

// send a byte to the LCD, RS pin must be setup before calling this function
static inline void lcd_sendByte(uint8_t data)
{
	lcd_sendNybble((data & 0xF0) >> 4);
	lcd_sendNybble((data & 0x0F) >> 0);
	
	// minimum time for execution
	_delay_us(50);
}

// display a character on the LCD
void lcd_sendChar(char c)
{
	// displaying a character means we turn on RS
	LCD_CTRL_PORTx |= _BV(LCD_CTRL_PINNUM_RS);

	lcd_sendByte(c);
	
	lcd_curX++;
}

// send a command to the LCD
void lcd_sendCmd(uint8_t data)
{
	// command means we turn off RS
	LCD_CTRL_PORTx &= ~_BV(LCD_CTRL_PINNUM_RS);

	lcd_sendByte(data);
}

// sets the position of the cursor, x is 0 indexed, line is not
void lcd_setPos(uint8_t x, uint8_t line)
{
	if (line == 1)
	{
		lcd_sendCmd(0b10000000 | 0x00 | x);
	}
	else if (line == 2)
	{
		lcd_sendCmd(0b10000000 | 0x40 | x);
	}

	lcd_curX = x;
	lcd_curY = line;

	// TODO support for 4 line displays needs to be added
}

// initialize the LCD
void lcd_init()
{
	// setup the pins
	DDRB |= _BV(4) | _BV(5) | _BV(6);
	DDRC |= _BV(6);
	PORTB &= ~(_BV(4) | _BV(5) | _BV(6));
	PORTC &= ~_BV(6);
	
	LCD_CTRL_DDRx |= _BV(LCD_CTRL_PINNUM_E) | _BV(LCD_CTRL_PINNUM_RS);
	LCD_CTRL_PORTx &= ~_BV(LCD_CTRL_PINNUM_E);
	LCD_CTRL_PORTx &= ~_BV(LCD_CTRL_PINNUM_RS);
	
	_delay_ms(40); // power on delay
	
	// clear the display
	lcd_sendCmd(0b00000001);
	_delay_us(1600);
	
	// function set, enable 4 bit mode first
	lcd_sendCmd(0b00101000);

	// display control: on, no cursor, no cursor blink
	lcd_sendCmd(0b00001100);
	// entry mode set: left to right, no display shift
	//lcd_sendCmd(0b00000110); // commented out because it's already default

	// start at line 1 position 0
	lcd_setPos(0, 1);
}

// provide STDIO
static int lcd_putchar(char c, FILE *stream)
{
	if (c == '\n') {
		// go to next line
		lcd_setPos(lcd_curX, lcd_curY == 1 ? 2 : 1);
	}
	else if (c == '\r') {
		// return to beginning of line
		lcd_setPos(0, lcd_curY);
	}
	else if (c == '\f') {
		// clear rest of line
		while (lcd_curX < 8) {
			lcd_sendChar(' ');
		}
	}
	else {
		// anything else, just print it
		lcd_sendChar(c);
	}
	return 0;
}

FILE lcd_stdout = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE);