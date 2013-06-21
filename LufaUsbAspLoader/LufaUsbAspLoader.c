/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the LufaUsbAspLoader bootloader. This file contains the complete bootloader logic.
 *  This is created by combining BootloaderHID from the LUFA project and USBasp from V-USB
 *  The combination is done by Frank26080115
 *
 *  This particular edition is used by Frank's Constant Current Dummy Load (Digital & USB)
 *
 */

#include "LufaUsbAspLoader.h"

// these definitions are taken directly from the original USBasp code
#define USBASP_FUNC_CONNECT         1
#define USBASP_FUNC_DISCONNECT      2
#define USBASP_FUNC_TRANSMIT        3
#define USBASP_FUNC_READFLASH       4
#define USBASP_FUNC_ENABLEPROG      5
#define USBASP_FUNC_WRITEFLASH      6
#define USBASP_FUNC_READEEPROM      7
#define USBASP_FUNC_WRITEEEPROM     8
#define USBASP_FUNC_SETLONGADDRESS  9

/* defaults if not in config file: */
#ifndef HAVE_EEPROM_PAGED_ACCESS
#   define HAVE_EEPROM_PAGED_ACCESS 0
#endif
#ifndef HAVE_EEPROM_BYTE_ACCESS
#   define HAVE_EEPROM_BYTE_ACCESS  0
#endif
#ifndef BOOTLOADER_CAN_EXIT
#   define  BOOTLOADER_CAN_EXIT     0
#endif

#if (FLASHEND) > 0xffff /* we need long addressing */
#   define CURRENT_ADDRESS  currentAddress.l
#   define addr_t           uint64_t
#else
#   define CURRENT_ADDRESS  currentAddress.w[0]
#   define addr_t           uint16_t
#endif

// USBasp datatypes

typedef union longConverter{
    addr_t  l;
    uint16_t    w[sizeof(addr_t)/2];
    uint8_t   b[sizeof(addr_t)];
}longConverter_t;

typedef union usbWord{
    uint16_t    word;
    uint8_t       bytes[2];
}usbWord_t;

typedef struct usbRequest{
    uint8_t       bmRequestType;
    uint8_t       bRequest;
    usbWord_t   wValue;
    usbWord_t   wIndex;
    usbWord_t   wLength;
}usbRequest_t;

// USBaspLoader global variables, these needs to be volatile or else something screws up and the code won't bootload right

static volatile longConverter_t  currentAddress; /* in bytes */
static volatile uint8_t          bytesRemaining;
static volatile uint8_t          isLastPage;
static volatile uint8_t          currentRequest = 0;

static const uint8_t  signatureBytes[4] = {
    SIGNATURE_0, SIGNATURE_1, SIGNATURE_2, 0
};

// prototypes
uint8_t usbFunctionWrite(uint8_t *data, uint8_t len);
uint8_t usbFunctionRead(uint8_t *data, uint8_t len);

// call this to start user application
void (*app_start)(void) = 0x0000;

/** Flag to indicate if the bootloader should be running, or should exit and allow the application code to run
 *  via a soft reset. When cleared, the bootloader will abort, the USB interface will shut down and the application
 *  started via a forced watchdog reset.
 */
static volatile bool RunBootloader = true;

/** Main program entry point. This routine configures the hardware required by the bootloader, then continuously
 *  runs the bootloader processing routine until instructed to soft-exit.
 */
int main(void)
{
    /* Setup hardware required for the bootloader */
    SetupHardware();

    /* Enable global interrupts so that the USB stack can function */
    sei();

    while (RunBootloader)
    {
        USB_USBTask();
    }

    /* Disconnect from the host - USB interface will be reset later along with the AVR */
    USB_Detach();

    /* Start User App */
    cli();
    MCUCR = (1 << IVCE);     /* enable change of interrupt vectors */
    MCUCR = (0 << IVSEL);    /* move interrupts to application flash section */
    app_start();
}

/** Configures all hardware required for the bootloader. */
static void SetupHardware(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();
    
    /* 16 MHz XTAL but uses 3.3V means it's better to run at 8 MHz */
    clock_prescale_set(clock_div_2);
    
    // this is an explicit check on the HWB pin, not really required, but my particular project needs it
    // note: on my circuit, there is an external pull up resistor on HWB
    DDRE &= ~_BV(2); PORTE &= ~_BV(2);
    if (bit_is_set(PINE, 2)) { app_start(); }
    // start user app if we shouldn't activate bootloader

    /* Relocate the interrupt vector table to the bootloader section */
    MCUCR = (1 << IVCE);
    MCUCR = (1 << IVSEL);

    /* Initialize USB subsystem */
    USB_Init();
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
    if ((USB_ControlRequest.bmRequestType & 0x60) != REQTYPE_VENDOR) return; // let LUFA internal code handle this
    
    if (USB_ControlRequest.bmRequestType != (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQREC_DEVICE) && USB_ControlRequest.bmRequestType != (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_DEVICE))
    {
        // hmm, this shouldn't happen, just clear it
        Endpoint_ClearStatusStage();
        return;
    }
    
    // code below is copied from USBaspLoader
    
    usbRequest_t    *rq = (usbRequest_t *)(&USB_ControlRequest); // typecast convert from LUFA datatype to V-USB datatype
    uint8_t         len = 0;
    static uint8_t    replyBuffer[512];

    if(rq->bRequest == USBASP_FUNC_TRANSMIT){   /* emulate parts of ISP protocol */
        uint8_t rval = 0;
        usbWord_t address;
        address.bytes[1] = rq->wValue.bytes[1];
        address.bytes[0] = rq->wIndex.bytes[0];
        if(rq->wValue.bytes[0] == 0x30){        /* read signature */
            rval = rq->wIndex.bytes[0] & 3;
            rval = signatureBytes[rval];
#if HAVE_EEPROM_BYTE_ACCESS
        }else if(rq->wValue.bytes[0] == 0xa0){  /* read EEPROM byte */
            rval = eeprom_read_byte((void *)address.word);
        }else if(rq->wValue.bytes[0] == 0xc0){  /* write EEPROM byte */
            eeprom_write_byte((void *)address.word, rq->wIndex.bytes[1]);
#endif
#if HAVE_CHIP_ERASE
        }else if(rq->wValue.bytes[0] == 0xac && rq->wValue.bytes[1] == 0x80){  /* chip erase */
            addr_t addr;
            for(addr = 0; addr < FLASHEND + 1 - (2048 * 2); addr += SPM_PAGESIZE) {
                /* wait and erase page */
#   ifndef NO_FLASH_WRITE
                boot_spm_busy_wait();
                boot_page_erase(addr);
#   endif
            }
#endif
        }else{
            /* ignore all others, return default value == 0 */
        }
        replyBuffer[3] = rval;
        len = 4;
    }else if(rq->bRequest == USBASP_FUNC_ENABLEPROG){
        replyBuffer[0] = 0;
        /* replyBuffer[0] = 0; is never touched and thus always 0 which means success */
        len = 1;
    }else if(rq->bRequest >= USBASP_FUNC_READFLASH && rq->bRequest <= USBASP_FUNC_SETLONGADDRESS){
        currentAddress.w[0] = rq->wValue.word;
        if(rq->bRequest == USBASP_FUNC_SETLONGADDRESS){
#if (FLASHEND) > 0xffff
            currentAddress.w[1] = rq->wIndex.word;
#endif
        len = 1;
        replyBuffer[0] = 0; // probably not needed
        }else{
            bytesRemaining = rq->wLength.bytes[0];
            /* if(rq->bRequest == USBASP_FUNC_WRITEFLASH) only evaluated during writeFlash anyway */
            isLastPage = rq->wIndex.bytes[1] & 0x02;
#if HAVE_EEPROM_PAGED_ACCESS
            currentRequest = rq->bRequest;
#endif
            len = 0xff; /* hand over to usbFunctionRead() / usbFunctionWrite() */
            
        }
#if BOOTLOADER_CAN_EXIT
    }else if(rq->bRequest == USBASP_FUNC_DISCONNECT){
        RunBootloader = false;      /* allow proper shutdown/close of connection */
        len = 1;
        replyBuffer[0] = 0; // probably not needed
#endif
    }else{
        /* ignore: USBASP_FUNC_CONNECT */
        len = 1;
        replyBuffer[0] = 0; // probably not needed
    }
    
    // the code below pretends to be the stuff that happens after usbFunctionSetup returns "len"
    
    if (len > 0)
    {
        Endpoint_ClearSETUP(); // ClearSETUP required here or else AVRDUDE gives "error: programm enable: target doesn't answer. 0"
        Endpoint_SelectEndpoint(0);
        
        if ((USB_ControlRequest.bmRequestType & 0x80) == REQDIR_DEVICETOHOST)
        {
            while (Endpoint_IsINReady() == 0);
            
            if (len == 0xFF) // this means either usbFunctionRead or usbFunctionWrite is needed, but now we want device to host, so we run usbFunctionRead
            {
                len = usbFunctionRead(replyBuffer, rq->wLength.bytes[0]);
                Endpoint_Write_Control_Stream_LE(replyBuffer, len);
                Endpoint_ClearOUT();
            }
            else
            {
                // len != 0xFF means send the replyBuffer without usbFunctionRead          
                Endpoint_Write_Control_Stream_LE(replyBuffer, len);
                Endpoint_ClearOUT();
            }
        }
        else // HOST to DEVICE
        {
            if (len == 0xFF) // this means either usbFunctionRead or usbFunctionWrite is needed, but now we want device to host, so we run usbFunctionWrite
            {
                for ( ; ; ) // this loop keeps feeding usbFunctionWrite until the last Byte is received
                {
                    len = Endpoint_BytesInEndpoint(); // check how many to feed
                    if (len != 0)
                    {
                        Endpoint_Read_Control_Stream_LE(replyBuffer, len);
                        uint8_t isLast = usbFunctionWrite(replyBuffer, len);
                        if (isLast)
                        {
                            // last byte received, finish up
                            Endpoint_ClearStatusStage();
                            break;
                        }
                    }
                }
            }
            else
            {
                // this should never be reached
                Endpoint_ClearStatusStage();
            }
        }
    }
}

// code below is copied from USBaspLoader

uint8_t usbFunctionWrite(uint8_t *data, uint8_t len)
{
    uint8_t   isLast;

    if(len > bytesRemaining)
        len = bytesRemaining;
    bytesRemaining -= len;
    isLast = bytesRemaining == 0;
    if(currentRequest >= USBASP_FUNC_READEEPROM){
        uint8_t i;
        for(i = 0; i < len; i++){
            eeprom_write_byte((void *)(currentAddress.w[0]++), *data++);
        }
    }else{
        uint8_t i;
        for(i = 0; i < len;){
#if !HAVE_CHIP_ERASE
            if((currentAddress.w[0] & (SPM_PAGESIZE - 1)) == 0){    /* if page start: erase */
#   ifndef NO_FLASH_WRITE
                boot_page_erase(CURRENT_ADDRESS);   /* erase page */
                boot_spm_busy_wait();               /* wait until page is erased */
#   endif
            }
#endif
            i += 2;
            boot_page_fill(CURRENT_ADDRESS, *(short *)data);
            CURRENT_ADDRESS += 2;
            data += 2;
            /* write page when we cross page boundary or we have the last partial page */
            if((currentAddress.w[0] & (SPM_PAGESIZE - 1)) == 0 || (isLast && i >= len && isLastPage)){
#ifndef NO_FLASH_WRITE
                boot_page_write(CURRENT_ADDRESS - 2);
                boot_spm_busy_wait();
                boot_rww_enable();
#endif
            }
        }
    }
    return isLast;
}

uint8_t usbFunctionRead(uint8_t *data, uint8_t len)
{
    uint8_t   i;

    if(len > bytesRemaining)
        len = bytesRemaining;
    bytesRemaining -= len;
    for(i = 0; i < len; i++){
        if(currentRequest >= USBASP_FUNC_READEEPROM){
            *data = eeprom_read_byte((void *)currentAddress.w[0]);
        }else{
            *data = pgm_read_byte((void *)CURRENT_ADDRESS);
        }
        data++;
        CURRENT_ADDRESS++;
    }
    return len;
}
