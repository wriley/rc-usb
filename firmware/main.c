/* Name: main.c
 * Project: rc-usb, a sime HID joystick
 * Author: William Riley
 * Creation Date: 2009-07-14
 * Tabsize: 4
 * Copyright: (c) 2009 by William Riley
 * License: GNU GPL v2 (see License.txt)
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#include "usbdrv.h"
#include "oddebug.h"

#define MAX_CHANNELS	8			// total number of A/D channels

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[50] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              // LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    // REPORT_SIZE (8)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x82,                    //     INPUT (Data,Var,Abs,Vol)
    0xc0,                          //   END_COLLECTION
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x32,                    //     USAGE (Z)
    0x09, 0x33,                    //     USAGE (Rx)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x82,                    //     INPUT (Data,Var,Abs,Vol)
    0xc0,                          //   END_COLLECTION
    0x09, 0x34,                    //   USAGE (Ry)
    0x09, 0x35,                    //   USAGE (Rz)
    0x09, 0x36,                    //   USAGE (Slider)
    0x09, 0x37,                    //   USAGE (Dial)
    0x95, 0x04,                    //   REPORT_COUNT (4)
    0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
    0xc0                           // END_COLLECTION
};

int  channelData[MAX_CHANNELS];
char newDataFound;

// USB report buffer
static uchar usbReport[MAX_CHANNELS];


//
// Build the USB report data using channel data in uS
//
// Calculates each report byte using the following:
//   - source data should be 732..2268 uS
//   - it should result in 0..255 byte values
//
static uchar *usbBuildReport(void)
{
    // rebuild the report only if new data was captured
    if (newDataFound)
    {
        int  *dp = &channelData[0];     // channel data pointer
        uchar *rp = &usbReport[0];      // report buffer pointer
        int data;
        char i;

        for (i = 0; i < MAX_CHANNELS; i++)
        {
            // ensure the atomic operation
            cli();
            data = *dp++;
            sei();

            // bounds check and correction
            if (data <  (1500-6*128)) data = (1500-6*128);
            if (data >= (1500+6*128)) data = (1500+6*128)-1;

            // storing channel
            *rp++ = (uchar)((data - (1500-6*128)) / 6);
        }

        // reset new data flag
        newDataFound = 0;
    }

    // return the address of buffer
    return &usbReport[0];
}

//-----------------------------------------------------------------------------

//
// USB setup request processing
//
uchar usbFunctionSetup(uchar data[8])
{
    usbRequest_t *rq = (void *)data;

    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS)
    {
        // Return the report if host requests it without USB interrupt
        if (rq->bRequest == USBRQ_HID_GET_REPORT)
        {
            // wValue: ReportType (highbyte), ReportID (lowbyte)
            // we only have one report type, so don't look at wValue
            usbMsgPtr = usbBuildReport();
            return sizeof(usbReport);
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------

//
// Check if the USB Interrupt In point buffer is empty and return
// the data buffer for the following host request via USB interrupt
//
void outSendData(void)
{
    if (usbInterruptIsReady())
    {
        // fill in the report buffer and return the
        // data pointer (Report ID is not used)
        usbSetInterrupt(usbBuildReport(), sizeof(usbReport));
    }
}

/* ------------------------------------------------------------------------- */

void inDecoderPoll(void)
{
	for (char i = 0; i < (MAX_CHANNELS - 1); i++)
    {
        ADMUX &= ~0x1f;             // ADMUX mask
        ADMUX |= i;                 // select ADC channel
        ADCSR |= (1<<ADSC);         // start conversion
        while (ADCSR & (1<<ADSC)) ; // wait for result (fast, not a problem for USB driver)
        int adc = ADC;                  // read the result
        int data = adc + (1500-1024/2); // convert to uS
        channelData[i] = data;          // save to data buffer
    }

	channelData[7] = 0;

	// set new data flag
    newDataFound = MAX_CHANNELS;
}

void inDecoderInit()
{
	// configure ADC
    ADMUX  = (0<<REFS1)|(1<<REFS0);                         // AREF=AVCC
    ADCSR  = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    // lowest frequency
}

void wdInit(void)
{
#if defined(__IOM16_H) || defined(__IOM32_H)
    WDTCR = (7 << WDP0) | (1 << WDE);
#else
    WDTCR = (1 << WDCE) | (1 << WDE);
#endif
    WDTCR = (7 << WDP0) | (1 << WDE);
}

void wdReset(void)
{
    wdt_reset();
}

int main(void)
{
    wdInit();                       // initialize watchdog timer
    usbInit();                      // initialize USB driver
    inDecoderInit();                // initialize input decoder
    usbDeviceConnect();             // connect USB device to USB bus
    sei();                          // required by USB driver and some interfaces

    while (1)
    {
        wdReset();                  // reset watchdog timer
        inDecoderPoll();            // poll for input data
        outSendData();              // prepare data for USB Interrupt In endpoint
        usbPoll();                  // process USB requests

	}
}

/* ------------------------------------------------------------------------- */
