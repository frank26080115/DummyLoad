#include <lcd.h>
#include <usb_serial.h>
#include <exadc.h>
#include <dac.h>
#include <i2clite.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// feature enable
#define ENABLE_ENC_PUSH
#define ENABLE_ENC_LOCK // must enable PUSH as well
#define ENABLE_LEDS
#define ENABLE_VOLTAGE_READ
#define ENABLE_USB
#define ENABLE_CONSOLE
#define ENABLE_CURRENT_ERROR_WARN // this won't work unless LEDs are enabled

// hardware definitions

#define ENC_A_PORTx			PORTD
#define ENC_A_DDRx			DDRD
#define ENC_A_PINx			PIND
#define ENC_A_PINNUM		3
#define ENC_B_PORTx			PORTD
#define ENC_B_DDRx			DDRD
#define ENC_B_PINx			PIND
#define ENC_B_PINNUM		2
#define ENC_PUSH_PORTx		PORTE
#define ENC_PUSH_DDRx		DDRE
#define ENC_PUSH_PINx		PINE
#define ENC_PUSH_PINNUM		2
#define LED_BLUE_PORTx		PORTF
#define LED_BLUE_DDRx		DDRF
#define LED_BLUE_PINNUM		6
#define LED_ORANGE_PORTx	PORTF
#define LED_ORANGE_DDRx		DDRF
#define LED_ORANGE_PINNUM	5

#define EXADC_VREF			2.048d
#define DAC_VREF			3.3d
#define BITS_EXADC_MAX		32768.0d
#define BITS_DAC_MAX		4096.0d
#define RSENSE				0.5d
#define VDIVIDER_R8			100.0d
#define VDIVIDER_R9			15.0d
#define VSCALE_DEFAULT		(EXADC_VREF * (VDIVIDER_R8 + VDIVIDER_R9) / VDIVIDER_R9 / BITS_EXADC_MAX)
#define IADCSCALE_DEFAULT	(1000.0d * EXADC_VREF / BITS_EXADC_MAX / RSENSE)
#define IDACSCALE_DEFAULT	(1000.0d * DAC_VREF / BITS_DAC_MAX / RSENSE)

// software definitions

#define ENC_SPEED_MAX	256
#define ENC_SPEED_INC	(ENC_SPEED_MAX / 16)
#define ENC_SPEED_DEC	(ENC_SPEED_MAX / 4)

#define GET_RAW_VOLTAGE(x)			lround((((double)(x)) / calibration.voltageScale))
#define GET_REAL_VOLTAGE(x)			((double)(((double)(x)) * calibration.voltageScale))
#define GET_RAW_CURRENT_ADC(x)		lround((((double)(x)) / calibration.currentAdcScale))
#define GET_REAL_CURRENT_ADC(x)		((double)(((double)(x)) * calibration.currentAdcScale))
#define GET_RAW_CURRENT_DAC(x)		lround((((double)(x)) / calibration.currentDacScale))
#define GET_REAL_CURRENT_DAC(x)		((double)(((double)(x)) * calibration.currentDacScale))

#define CURRENT_ERROR_THRESHOLD	10 // in percent

#define FILTER_CONST	0.9d // used by the low pass filter, between 0 and 1 please

#define EXADC_CURRENT_CHAN 0
#define EXADC_VOLTAGE_CHAN 1

typedef struct  
{
	int16_t voltageOffset;
	int16_t currentOffset;
	double voltageScale;
	double currentAdcScale;
	double currentDacScale;
}
calibration_t;
#define CALIBRATION_EEPROM_ADDR ((const void*)(0))

// function declarations

static void spitData();
static void takeData();
static int32_t takeAverage(uint8_t, uint8_t);
static void encoderHandler();
static int ser_putchar(char, FILE*);

// global variables

static volatile uint8_t prevEnc = 0;
static char serOutputBuffer[32];
static char serInputBuffer[32];
static uint8_t serInputBufferIdx = 0;
static char isOutputting = 0;
static int16_t rawVoltage, rawCurrent;
static double rawFilteredVoltage, rawFilteredCurrent;
static int16_t setRawCurrent = 0;
static int16_t encSpeed = 0;
static char encDoNotDec = 0;
static uint8_t showSetTimer = 0;
static uint32_t spitCnt;
static char isLocked = 0;
static volatile uint8_t encFlags = 0;
static volatile int encHasAction = 0;
static uint8_t pushHoldCnt = 0;
static volatile char tmr1OvfFlag = 0;
static volatile char tmr1OvfCnt = 0;
static calibration_t calibration;

static FILE ser_stdout = FDEV_SETUP_STREAM(ser_putchar, NULL, _FDEV_SETUP_WRITE);
#define ser_printf(fmt, args...) fprintf(&ser_stdout, fmt, ##args)
#define ser_printf_P(fmt, args...) fprintf_P(&ser_stdout, PSTR(fmt), ##args)

int main(void)
{
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_2); // the circuit uses a 16 MHz crystal but this makes it run at 8 MHz, to make it work at 3.3V

	// port and pin setup
	// encoder inputs, with pull-up resistors
	ENC_A_DDRx &= ~_BV(ENC_A_PINNUM);
	ENC_B_DDRx &= ~_BV(ENC_B_PINNUM);
	ENC_A_PORTx |= _BV(ENC_A_PINNUM);
	ENC_B_PORTx |= _BV(ENC_B_PINNUM);
	#if defined(ENABLE_ENC_PUSH) || defined(ENABLE_ENC_LOCK)
	ENC_PUSH_DDRx &= ~_BV(ENC_PUSH_PINNUM);
	// ENC_PUSH_PORTx |= _BV(ENC_PUSH_PINNUM); // resistor is implemented externally
	#endif

	#ifdef ENABLE_LEDS
	// setup LEDs
	LED_BLUE_DDRx |= _BV(LED_BLUE_PINNUM);
	LED_ORANGE_DDRx |= _BV(LED_ORANGE_PINNUM);
	LED_BLUE_PORTx &= ~_BV(LED_BLUE_PINNUM);
	LED_ORANGE_PORTx &= ~_BV(LED_ORANGE_PINNUM);
	#endif

	// start timer, trigger every 0.01s
	TCCR1B = _BV(WGM12) | 0x02;
	OCR1A = 31250; // this should not be lower than 500
	TIMSK1 |= _BV(OCIE1A);

	// setup external interrupts on encoder inputs (INT2 and INT3), any edge
	EICRA = _BV(ISC30) | _BV(ISC20);
	EIMSK = _BV(INT3) | _BV(INT2);

	lcd_init();
	lcd_printf("Hello\f\r\nWorld\f");

	#ifdef ENABLE_USB
	usb_init();
	#else
	// TODO implement other serial IO
	#endif

	i2c_init();
	dac_init();
	exadc_init();

	// read in calibration from EEPROM here
	eeprom_read_block((void*)(&calibration), CALIBRATION_EEPROM_ADDR, sizeof(calibration_t));
	// then validate
	if (calibration.currentOffset < 0 || calibration.currentOffset > 0x7FFF) {
		calibration.currentOffset = 0;
	}
	if (calibration.voltageOffset < 0 || calibration.voltageOffset > 0x7FFF) {
		calibration.voltageOffset = 0;
	}
	if (calibration.voltageScale <= 0.0d || calibration.voltageScale > 1.0d || calibration.voltageScale != calibration.voltageScale) {
		calibration.voltageScale = VSCALE_DEFAULT;
	}
	if (calibration.currentAdcScale <= 0.0d || calibration.currentAdcScale > 1.0d || calibration.currentAdcScale != calibration.currentAdcScale) {
		calibration.currentAdcScale = IADCSCALE_DEFAULT;
	}
	if (calibration.currentDacScale <= 0.0d || calibration.currentDacScale > 1.0d || calibration.currentDacScale != calibration.currentDacScale) {
		calibration.currentDacScale = IDACSCALE_DEFAULT;
	}

	sei(); // ready to go? enable global interrupts!

	while(1)
	{
		// take a sample and filter every 0.01s
		if (tmr1OvfFlag != 0) {
			takeData();
			tmr1OvfFlag = 0;
		}

		if (tmr1OvfCnt >= 25 || encDoNotDec != 0)
		{
			// spit out data to USB, the counter gives each line a timestamp
			if (isOutputting != 0 && tmr1OvfCnt >= 25)
			{
				spitCnt++;
				if (spitCnt >= 0x7FFFFFFF) {
					spitCnt = 0;
				}
				spitData();
			}

			tmr1OvfCnt = tmr1OvfCnt >= 25 ? (tmr1OvfCnt - 25) : tmr1OvfCnt;

			// show info on LCD
			lcd_setPos(0, 1);
			#ifdef ENABLE_VOLTAGE_READ
			lcd_printf("%.4f\f", GET_REAL_VOLTAGE(rawFilteredVoltage));
			lcd_setPos(7, 1);
			lcd_printf("V");
			#endif

			// show the target current only for a brief period before going back to displaying the actual current
			#ifdef ENABLE_VOLTAGE_READ
			if (showSetTimer > 0)
			{
				if (encDoNotDec == 0) {
					showSetTimer--;
				}
				lcd_setPos(0, 2);
				lcd_printf(">%.3f\f", GET_REAL_CURRENT_DAC(setRawCurrent));
			}
			else
			#else
			lcd_setPos(0, 1);
			lcd_printf(">%.3f\f", GET_REAL_CURRENT_DAC(setRawCurrent));
			lcd_setPos(6, 1);
			lcd_printf("mA");
			#endif
			{
				lcd_setPos(0, 2);
				lcd_printf("%.4f\f", GET_REAL_CURRENT_ADC(rawFilteredCurrent));
			}
			lcd_setPos(6, 2);
			lcd_printf("mA");

			// dynamically adjust encoder increments
			if (encDoNotDec == 0) // but do not decrement the speed if recently adjusted
			{
				if (encSpeed > ENC_SPEED_DEC) {
					encSpeed -= ENC_SPEED_DEC;
				}
				else if (encSpeed > 0) {
					encSpeed = 0;
				}
			}
			else
			{
				encDoNotDec = 0;
			}

			#ifdef ENABLE_ENC_LOCK
			if (bit_is_set(pushHoldCnt, 7))
			{
				// this means the button is being held down
				if (pushHoldCnt < 0x88) {
					pushHoldCnt++;
				}
				else {
					// been held long enough
					pushHoldCnt = 0x40; // signal that release is pending 
					isLocked = isLocked ? 0 : 1; // toggle locked state
				}
			}
			#endif
		}

		#ifdef ENABLE_LEDS
		if (isLocked != 0) {
			LED_BLUE_PORTx |= _BV(LED_BLUE_PINNUM);
		}
		else {
			LED_BLUE_PORTx &= ~_BV(LED_BLUE_PINNUM);
		}

		double err = abs(GET_REAL_CURRENT_ADC(rawFilteredCurrent) - GET_REAL_CURRENT_DAC(setRawCurrent));
		err *= 100; // convert to percent
		err /= GET_REAL_CURRENT_DAC(setRawCurrent);
		if (err > ((double)(CURRENT_ERROR_THRESHOLD))) {
			LED_ORANGE_PORTx |= _BV(LED_ORANGE_PINNUM);
		}
		else {
			LED_ORANGE_PORTx &= ~_BV(LED_ORANGE_PINNUM);
		}
		#endif

		#ifdef ENABLE_CONSOLE
        // handle serial input
		int16_t c;
		#ifdef ENABLE_USB
		c = usb_serial_getchar();
		#else
		// TODO implement other input methods
		#endif
		if (c == 0 || c == '\r' || c == '\n')
		{ // command entered
			serInputBuffer[serInputBufferIdx] = 0; // null terminate
			if (strcmp(serInputBuffer, "start") == 0)
			{
				isOutputting = 1;
				spitCnt = 0;
			}
			else if (strcmp(serInputBuffer, "stop") == 0)
			{
				isOutputting = 0;
			}
			else if (strcmp(serInputBuffer, "single") == 0)
			{
				spitData();
			}
			else if (strcmp(serInputBuffer, "info") == 0)
			{
				ser_printf("currentOffset = %d\r\n", calibration.currentOffset);
				#ifdef ENABLE_VOLTAGE_READ
				ser_printf("voltageOffset = %d\r\n", calibration.voltageOffset);
				ser_printf("voltageScale = %.8f\r\n", calibration.voltageScale);
				#endif
				ser_printf("currentAdcScale = %.8f\r\n", calibration.currentAdcScale);
				ser_printf("currentDacScale = %.8f\r\n", calibration.currentDacScale);
			}
			else if (memcmp(serInputBuffer, "set:", 4) == 0 && serInputBufferIdx > 4)
			{
				char** np;
				double d = strtod(&serInputBuffer[4], &np);
				if (np != &serInputBuffer[4])
				{
					int16_t dd = GET_RAW_CURRENT_DAC(d);
					if (dd >= 0 && dd <= 4095) {
						dac_write(setRawCurrent = dd);
						showSetTimer = 4;
					}
					else {
						ser_printf("out of range\r\n");
					}
				}
				else
				{
					ser_printf("syntax error\r\n");
				}
			}
			else if (memcmp(serInputBuffer, "set16:", 6) == 0 && serInputBufferIdx > 6)
			{
				char** np;
				double d = strtod(&serInputBuffer[6], &np);
				if (np != &serInputBuffer[6])
				{
					int16_t dd = lround(d);
					if (dd >= 0 && dd <= 4095) {
						dac_write(setRawCurrent = dd);
						showSetTimer = 4;
					}
					else {
						ser_printf("out of range\r\n");
					}
				}
				else
				{
					ser_printf("syntax error\r\n");
				}
			}
			else if (memcmp(serInputBuffer, "echo:", 5) == 0)
			{
				ser_printf("%s\r\n", &(serInputBuffer[5]));
			}
			else if (memcmp(serInputBuffer, "testnum:", 8) == 0)
			{
				// this is purely to test whether or not you've compiled the floating point support properly
				ser_printf("%.8f\r\n", atof(&(serInputBuffer[8])));
			}
			else if (strcmp(serInputBuffer, "cali0amps") == 0)
			{
				dac_write(0);
				calibration.currentOffset = takeAverage(EXADC_CURRENT_CHAN, 20); // average with rounding
				eeprom_update_block((void*)(&calibration), CALIBRATION_EEPROM_ADDR, sizeof(calibration_t));
				ser_printf("currentOffset = %d\r\n", calibration.currentOffset);
			}
			else if (memcmp(serInputBuffer, "caliamps:", 9) == 0 && serInputBufferIdx > 9)
			{
				char** np;
				double d = strtod(&serInputBuffer[9], &np);
				if (np != &serInputBuffer[9])
				{
					if (d > 0.0d)
					{
						if (dac_last >= 100)
						{
							calibration.currentDacScale = d / (double)dac_last;
							calibration.currentAdcScale = d / (double)(takeAverage(EXADC_CURRENT_CHAN, 20));
							eeprom_update_block((void*)(&calibration), CALIBRATION_EEPROM_ADDR, sizeof(calibration_t));
							ser_printf("currentAdcScale = %.8f\r\n", calibration.currentAdcScale);
							ser_printf("currentDacScale = %.8f\r\n", calibration.currentDacScale);
						}
						else
						{
							ser_printf("error: bad current setting for performing calibration\r\n");
						}
						// enter the voltage across the sense resistor in mV
						// a good power supply must be connected

					}
					else if (d == 0.0d)
					{
						// or calibrate the offset
						dac_write(0);
						calibration.currentOffset = takeAverage(EXADC_CURRENT_CHAN, 20);
						eeprom_update_block((void*)(&calibration), CALIBRATION_EEPROM_ADDR, sizeof(calibration_t));
						ser_printf("currentOffset = %d\r\n", calibration.currentOffset);
					}
					else
					{
						ser_printf("out of range\r\n");
					}
				}
				else
				{
					ser_printf("syntax error\r\n");
				}

			}
			#ifdef ENABLE_VOLTAGE_READ
			else if (memcmp(serInputBuffer, "calivolts:", 10) == 0 && serInputBufferIdx > 10)
			{
				char** np;
				double d = strtod(&serInputBuffer[10], &np);
				if (np != &serInputBuffer[10])
				{
					if (d < 8.8d && d > 0.0d) {
						dac_write(0);
						calibration.voltageScale = d / ((double)takeAverage(EXADC_VOLTAGE_CHAN, 20));
						eeprom_update_block((void*)(&calibration), CALIBRATION_EEPROM_ADDR, sizeof(calibration_t));
						ser_printf("voltageScale = %.8f\r\n", calibration.voltageScale);
					}
					else if (d == 0.0d) {
						dac_write(0);
						calibration.voltageOffset = takeAverage(EXADC_VOLTAGE_CHAN, 20);
						eeprom_update_block((void*)(&calibration), CALIBRATION_EEPROM_ADDR, sizeof(calibration_t));
						ser_printf("voltageOffset = %d\r\n", calibration.voltageOffset);
					}
					else {
						ser_printf("out of range\r\n");
					}
				}
				else
				{
					ser_printf("syntax error\r\n");
				}

			}
			#endif
			else if (serInputBufferIdx > 0)
			{
				ser_printf("unknown\r\n");
			}

			serInputBufferIdx = 0;
		}
		else if (c > 0)
		{
			// place into buffer for processing later
			serInputBuffer[serInputBufferIdx] = c;
			if (serInputBufferIdx < 31) {
				serInputBufferIdx++;
			}
		}
		#endif

		if (encHasAction != 0)
		{
			encDoNotDec = 1;

			// handle encoder actions
			if (encSpeed == 0) {
				encSpeed++;
			}
			else if (encSpeed < ENC_SPEED_MAX) {
				encSpeed += ENC_SPEED_INC;
			}

			if (encHasAction > 0)
			{
				encHasAction = 0;

				if (isLocked == 0) {
					setRawCurrent += encSpeed;
				}

				if (setRawCurrent > 4095) {
					setRawCurrent = 4095;
				}

				dac_write(setRawCurrent);

				showSetTimer = 4;
			}
			else if (encHasAction < 0)
			{
				encHasAction = 0;

				if (isLocked == 0) {
					setRawCurrent -= encSpeed;
				}

				if (setRawCurrent < 0) {
					setRawCurrent = 0;
				}

				dac_write(setRawCurrent);

				showSetTimer = 4;
			}
		}

		#ifdef ENABLE_ENC_PUSH
		// if somebody wants to see the setting instead of reading but without having to change it, push the button
		if (bit_is_clear(ENC_PUSH_PINx, ENC_PUSH_PINNUM))
		{
			showSetTimer = 4;
			if (bit_is_clear(pushHoldCnt, 6)) {
				// only if release is not pending
				pushHoldCnt |= 0x80; // signal that it is being held, checked in the 0.25s event
			}
		}
		else
		#endif
		{
			#ifdef ENABLE_ENC_LOCK
			pushHoldCnt = 0;
			#endif
		}
    }
}


static int ser_putchar(char c, FILE* f)
{
	// this function is here so printf can be used with USB
	#ifdef ENABLE_USB
	usb_serial_putchar(c);
	if (c == '\n') {
		usb_serial_flush_output(); // end of line so send out the entire output buffer
	}
	#endif
	// TODO implement other serial IO
	return 0;
}

static int32_t takeAverage(uint8_t chan, uint8_t cnt)
{
	int32_t sum = 0;
	for (uint8_t i = 0; i < cnt; i++)
	{
		_delay_ms(8);
		sum += exadc_read(chan);
	}
	return (sum + (cnt / 2)) / cnt; // this rounds
}

static void takeData()
{
	// take the readings
	rawCurrent = exadc_read(EXADC_CURRENT_CHAN);
	#ifdef ENABLE_VOLTAGE_READ
	rawVoltage = exadc_read(EXADC_VOLTAGE_CHAN);
	#endif

	// apply the offset
	rawCurrent -= calibration.currentOffset;
	rawVoltage -= calibration.voltageOffset;

	// keep within valid range
	if (rawCurrent < 0) rawCurrent = 0;
	if (rawVoltage < 0) rawVoltage = 0;

	// apply low pass filter
	rawFilteredCurrent = rawFilteredCurrent * FILTER_CONST + (1.0d - FILTER_CONST) * (double)rawCurrent;
	rawFilteredVoltage = rawFilteredVoltage * FILTER_CONST + (1.0d - FILTER_CONST) * (double)rawVoltage;
}

static void spitData()
{
	// convert time into seconds
	double t = spitCnt;
	t /= 4;

	ser_printf("%.2f,\t", t);
	#ifdef ENABLE_VOLTAGE_READ
	ser_printf("%.6f,\t%d,\t", GET_REAL_VOLTAGE(rawVoltage), rawVoltage);
	#endif
	ser_printf("%.6f,\t%d,\t", GET_REAL_CURRENT_ADC(rawCurrent), rawCurrent);
	ser_printf("%.6f,\t%d,\r\n", GET_REAL_CURRENT_DAC(setRawCurrent), setRawCurrent);
}

static void encoderHandler()
{
	// handle encoder movements
	uint8_t curEnc = 0x80; // the MSB indicates that it's been initialized
	if (bit_is_clear(ENC_A_PINx, ENC_A_PINNUM)) {
		curEnc |= 1 << 0;
	}
	if (bit_is_clear(ENC_B_PINx, ENC_B_PINNUM)) {
		curEnc |= 1 << 1;
	}
	if (prevEnc == 0)
	{
		// uninitialized previous state, so initialize it here
		prevEnc = curEnc;
	}
	else if (curEnc != prevEnc)
	{
		if (prevEnc == 0x80)
		{
			// determine the first edge
			if (curEnc == 0x81) {
				encFlags |= _BV(0);
			}
			else if (curEnc == 0x82) {
				encFlags |= _BV(1);
			}
		}
		if (curEnc == 0x83)
		{
			// middle state has been reached
			encFlags |= _BV(4);
		}
		else if (curEnc == 0x80)
		{
			// determine the final edge
			if (prevEnc == 0x82) {
				encFlags |= _BV(2);
			}
			else if (prevEnc == 0x81) {
				encFlags |= _BV(3);
			}

			// check the first and last edge
			// or maybe one edge is missing, if missing then require the middle state
			// this will reject bounces and false movements
			if (bit_is_set(encFlags, 0) && (bit_is_set(encFlags, 2) || bit_is_set(encFlags, 4))) {
				encHasAction = 1;
			}
			else if (bit_is_set(encFlags, 2) && (bit_is_set(encFlags, 0) || bit_is_set(encFlags, 4))) {
				encHasAction = 1;
			}
			else if (bit_is_set(encFlags, 1) && (bit_is_set(encFlags, 3) || bit_is_set(encFlags, 4))) {
				encHasAction = -1;
			}
			else if (bit_is_set(encFlags, 3) && (bit_is_set(encFlags, 1) || bit_is_set(encFlags, 4))) {
				encHasAction = -1;
			}

			encFlags = 0;
		}

		prevEnc = curEnc;
	}
}

ISR(INT2_vect)
{
	encoderHandler();
}

ISR(INT3_vect)
{
	encoderHandler();
}

// this happens every 0.25s
ISR(TIMER1_COMPA_vect)
{
	tmr1OvfFlag = 1; // signal main thread
	tmr1OvfCnt++;
}