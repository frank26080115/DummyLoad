Full project details at:

http://www.frank-zhao.com/dummyload/

The circuit and PCB are designed in EAGLE 6.3.0

Compiling the bootloader requires LUFA to be installed (not included, get it yourself)

The firmware is compiled using Atmel Studio 6.1, with the accompanying AVRToolChain, which has AVR GCC 4.7.2

The box is a Hammond Mfg RL6115

===================================

Instructions:

Build the circuit, the PCB and schematic are both provided, so it's up to you to find the parts and get the PCB made and then solder everything.

The box needs to be drilled and cut. I used a drill, a dremel, and some files to get this done.

After the circuit is built, use an AVR programmer to flash the bootloader into the ATmega32U4

To activate the bootloader, hold down the rotary encoder push button, and plug in the USB cable.

The bootloader behaves as a USBasp, use AVRDUDE to bootload the firmware.

Adjust the LCD contrast using the potentiometer.

The virtual USB serial port requires a driver, see http://www.pjrc.com/teensy/td_download.html and look for "Windows Serial Driver"

When you first use the dummy load, you must connect it to a computer and open up the virtual serial port to do the calibration.

===================================

Calibration:

This is a measurement tool. Because of the various errors and tolerances of the components being used, it needs to be calibrated before being useful. Calibration is done against a known "calibrated" power supply, which must have adjustable voltage, with both voltage and current readouts.

Make sure the virtual serial port is open in a serial terminal.

Connect the red plug to the black plug.

Type the command "calivolts:0" and press enter, wait until you see "voltageOffset" being shown.

Connect the dummy load to the power supply that was discussed previously.

Set the precision power supply to exactly 5V

Type the command "calivolts:5" and press enter, wait until you see "voltageScale" being shown. If you are not using 5V, you can type in another voltage.

Type the command "caliamps:0" and press enter, wait until you see "currentOffset" being shown.

Rotate the knob until the power supply says it is supplying exactly 500mA, then type the command "caliamps:500" and press enter, wait until you see "currentDacScale" and "currentAdcScale" being shown.

If you cannot get exactly 500mA, you can actually type in any number equal to the current being drawn.

All calibration values are stored in EEPROM. There are default values if the values are invalid or missing in the EEPROM.

====================================

Other Serial Terminal Commands

All of these commands are lower case, case sensitive, and must end with one of \r \n or \0, you can hit enter to do this.

"start" will start the logging output in comma separated value format

"stop" will stop the logging output

"single" will output only one line of measurement

"set:123.45" will set the current to 123.45 mA, you can specify any number you want

"set16:12345" will send 12345 to the DAC, this is only used for testing

"info" will report the calibration values

There are a few more internal testing commands that you do not need to know, see the source code if you are interested.

I recommend using RealTerm because it has a simple interface for logging to a file.