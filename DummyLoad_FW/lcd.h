#ifndef lcd_h
#define lcd_h

#include <stdio.h>

// display a character on the LCD
void lcd_sendChar(char c);

// send a command to the LCD
void lcd_sendCmd(uint8_t data);

// sets the position of the cursor, x is 0 indexed, line is not
void lcd_setPos(uint8_t x, uint8_t line);

// initialize the LCD
void lcd_init();

// provide STDIO
extern FILE lcd_stdout;
#define lcd_printf(fmt, args...) fprintf(&lcd_stdout, fmt, ##args)
#define lcd_printf_P(fmt, args...) fprintf_P(&lcd_stdout, PSTR(fmt), ##args)

#endif