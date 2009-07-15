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

#define MAX_CHANNELS	5			// total number of A/D channels
#define N_REPORT_IDS    2           // total number of Report IDs

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[64] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              // LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    // REPORT_SIZE (8)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    // REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x82,                    //     INPUT (Data,Var,Abs,Vol)
    0xc0,                          //     END_COLLECTION
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x09, 0x32,                    //     USAGE (Z)
    0x09, 0x33,                    //     USAGE (Rx)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x82,                    //     INPUT (Data,Var,Abs,Vol)
    0xc0,                          //     END_COLLECTION
    0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x34,                    //   USAGE (Ry)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //   USAGE_MAXIMUM (Button 8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
};

int  channelData[MAX_CHANNELS];

// USB report buffer
static uchar usbReport[8];          // cannot use more than 8 bytes for low-speed USB device


//
// Build the USB report data using channel data in uS and ReportID
// as an argument. If ReportID is 0 then select it automatically.
//
// Calculates each report byte using the following:
//   - source data should be 732..2268 uS
//   - it should result in 0..255 byte values
// Buttons reflect channel values as discrete controls.
//
static uchar usbBuildReport(uchar id)
{
    static uchar lastReport = 0;    // last ReportID sent

    // choose the ReportID to build: use specified or select next
    if (id)
        lastReport = id;
    else
        lastReport = (lastReport % N_REPORT_IDS) + 1;

    // preserve the ReportID
    uchar *rp = &usbReport[0];
    *rp++ = lastReport;

    // build the report requested
    int data;
    char i;

    switch (lastReport)
    {
        case 1:
            for (i = 0; i < 4; i++)
            {
                // ensure atomic operation
                cli();
                data = channelData[i];
                sei();

                // bounds check and correction
                if (data <  (1500-6*128)) data = (1500-6*128);
                if (data >= (1500+6*128)) data = (1500+6*128)-1;

                // storing channel
                *rp++ = (uchar)((data - (1500-6*128)) / 6);
            }

            // return size of Report 1
            return (1+4);

        case 2:
            for (i = 4; i < MAX_CHANNELS; i++)
            {
                // ensure atomic operation
                cli();
                data = channelData[i];
                sei();

                // bounds check and correction
                if (data <  (1500-6*128)) data = (1500-6*128);
                if (data >= (1500+6*128)) data = (1500+6*128)-1;

                // storing channel
                *rp++ = (uchar)((data - (1500-6*128)) / 6);
            }

            // prepare buttons
            char buttons = 0x00;

            *rp = buttons;

            // return size of Report 2
            return (1+1+1);
    }

    return 0;
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
            // build report requested
            usbMsgPtr = usbReport;
            return usbBuildReport(rq->wValue.bytes[0]);
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
        // data pointer (Report ID is selected automatically)
        usbSetInterrupt(usbReport, usbBuildReport(0));
    }
}

/* ------------------------------------------------------------------------- */

void inDecoderPoll(void)
{
	for (char i = 0; i < MAX_CHANNELS; i++)
    {
        ADMUX &= ~0x1f;             // ADMUX mask
        ADMUX |= i;                 // select ADC channel
        ADCSR |= (1<<ADSC);         // start conversion
        while (ADCSR & (1<<ADSC)) ; // wait for result (fast, not a problem for USB driver)
        int adc = ADC;                  // read the result
        int data = adc + (1500-1024/2); // convert to uS
        channelData[i] = data;          // save to data buffer
    }
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
