/* Name: main.c
 * Project: HID-Test
 * Author: Christian Starkjohann
 * Creation Date: 2006-02-02
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * This Revision: $Id$
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "usbdrv.h"
#include "oddebug.h"

/* ----------------------- hardware I/O abstraction ------------------------ */

#define LED_ON() do{PORTB|=0x02;}while(0)
#define LED_OFF() do{PORTB&=~0x02;}while(0)

/* pin assignments:
PB0	Key 1
PB1	Key 2
PB2	Key 3
PB3	Key 4
PB4	Key 5
PB5 Key 6

PC0	Key 7
PC1	Key 8
PC2	Key 9
PC3	Key 10
PC4	Key 11
PC5	Key 12

PD0	USB-
PD1	debug tx
PD2	USB+ (int0)
PD3	Key 13
PD4	Key 14
PD5	Key 15
PD6	Key 16
PD7	Key 17
*/

static void hardwareInit(void)
{
uchar	i, j;

    PORTB = 0xff;   /* activate all pull-ups */
    DDRB = 0;       /* all pins input */
    PORTC = 0xff;   /* activate all pull-ups */
    DDRC = 0;       /* all pins input */
    PORTD = 0xf3;   /* 1111 0011 bin: activate pull-ups except on USB lines */
    DDRD = 0x0a;    /* 0000 1100 bin: all pins input except USB (-> USB reset) */
	j = 0;
	while(--j){     /* USB Reset by device only required on Watchdog Reset */
		i = 0;
		while(--i); /* delay >10ms for USB reset */
	}
    DDRD = 0x02;    /* 0000 0010 bin: remove USB reset condition */
    /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
    TCCR0 = 5;      /* timer 0 prescaler: 1024 */

    LED_OFF(); // indicator led
    DDRB|=0x02;
}
/* ------------------------------------------------------------------------- */

/* The following function returns a bit value representing 8 keys (read at once)*/
/*static uint8_t scanKeys(void)
{
    static uint8_t lastKeysDetect=0x07, lastKeysRead=0x07;
    static uint8_t bounceCnt = 0;

    uint8_t keys = (PIND>>5);



    lastKeysRead = keys;
}*/

#define NUM_KEYS    18

/* The following function returns an index for the first key pressed. It
 * returns 0 if no key is pressed.
 */
static uchar    keyPressed(void)
{
    uchar   i, mask, x;

    /*x = PINB;
    mask = 1;
    for(i=0;i<6;i++){
        if((x & mask) == 0)
            return i + 1;
        mask <<= 1;
    }
    x = PINC;
    mask = 1;
    for(i=0;i<6;i++){
        if((x & mask) == 0)
            return i + 7;
        mask <<= 1;
    }*/

    x = PIND;
    mask = 1 << 4;
    for(i=0;i<4;i++){
        if((x & mask) == 0)
            return i + 13;
        mask <<= 1;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

const PROGMEM char usbHidReportDescriptor[35] = {   /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

#define KEY__ 0
#define KEY_errorRollOver 1
#define KEY_POSTfail 2
#define KEY_errorUndefined 3

#define KEY_A 4
#define KEY_B 5
#define KEY_C 6
#define KEY_D 7
#define KEY_E 8
#define KEY_F 9
#define KEY_G 10
#define KEY_H 11
#define KEY_I 12
#define KEY_J 13
#define KEY_K 14
#define KEY_L 15
#define KEY_M 16
#define KEY_N 17
#define KEY_O 18
#define KEY_P 19
#define KEY_Q 20
#define KEY_R 21
#define KEY_S 22
#define KEY_T 23
#define KEY_U 24
#define KEY_V 25
#define KEY_W 26
#define KEY_X 27
#define KEY_Y 28
#define KEY_Z 29
#define KEY_1 30
#define KEY_2 31
#define KEY_3 32
#define KEY_4 33
#define KEY_5 34
#define KEY_6 35
#define KEY_7 36
#define KEY_8 37
#define KEY_9 38
#define KEY_0 39

#define KEY_enter 40
#define KEY_esc 41
#define KEY_bckspc 42
#define KEY_tab 43
#define KEY_spc 44
#define KEY_minus 45
#define KEY_equal 46
#define KEY_lbr 47      // [
#define KEY_rbr 48      // ]  -- 0x30
#define KEY_bckslsh 49  // \ (and |)
#define KEY_hash 50     // Non-US # and ~
#define KEY_smcol 51    // ; (and :)
#define KEY_ping 52     // ' and "
#define KEY_grave 53    // Grave accent and tilde
#define KEY_comma 54    // , (and <)
#define KEY_dot 55      // . (and >)
#define KEY_slash 56    // / (and ?)
#define KEY_cpslck 57   // capslock

#define KEY_F1 58
#define KEY_F2 59
#define KEY_F3 60
#define KEY_F4 61
#define KEY_F5 62
#define KEY_F6 63
#define KEY_F7 64       // 0x40
#define KEY_F8 65
#define KEY_F9 66
#define KEY_F10 67
#define KEY_F11 68
#define KEY_F12 69

#define KEY_PrtScr 70
#define KEY_scrlck 71
#define KEY_break 72
#define KEY_ins 73
#define KEY_home 74
#define KEY_pgup 75
#define KEY_del 76
#define KEY_end 77
#define KEY_pgdn 78
#define KEY_rarr 79
#define KEY_larr 80     // 0x50
#define KEY_darr 81
#define KEY_uarr 82
#define KEY_numlock 83
#define KEY_KPslash 84
#define KEY_KPast 85
#define KEY_KPminus 86
#define KEY_KPplus 87
#define KEY_KPenter 88
#define KEY_KP1 89
#define KEY_KP2 90
#define KEY_KP3 91
#define KEY_KP4 92
#define KEY_KP5 93
#define KEY_KP6 94
#define KEY_KP7 95
#define KEY_KP8 96      // 0x60
#define KEY_KP9 97
#define KEY_KP0 98
#define KEY_KPcomma 99

static const uchar  keyReport[NUM_KEYS + 1][2] PROGMEM = {
/* none */  {0, 0},                     /* no key pressed */
/*  1 */    {0, KEY_KP0},
/*  2 */    {0, KEY_KP1},
/*  3 */    {0, KEY_KP2},
/*  4 */    {0, KEY_KP3},
/*  5 */    {0, KEY_KP4},
/*  6 */    {0, KEY_KP5},
/*  7 */    {0, KEY_KP6},
/*  8 */    {0, KEY_KP7},
/*  9 */    {0, KEY_KP8},
/* 10 */    {0, KEY_KP9},
/* 11 */    {0, KEY_E},
/* 12 */    {0, KEY_KPminus},
/* 13 */    {0, KEY_KPcomma},
/* 14 */    {0, KEY_rarr},
/* 15 */    {0, KEY_larr},
/* 16 */    {0, KEY_darr},
/* 17 */    {0, KEY_uarr},
/* 18 */    {0, KEY_KPenter}
};

static void buildReport(uchar key)
{
/* This (not so elegant) cast saves us 10 bytes of program memory */
    *(int *)reportBuffer = pgm_read_word(keyReport[key]);
}

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    // class request type
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  // wValue: ReportType (highbyte), ReportID (lowbyte)
            // we only have one report type, so don't look at wValue
            buildReport(keyPressed());
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        // no vendor specific requests implemented
    }
	return 0;
}

/* --- circular buffer ----------------------------------------------------- */

#define KEYBUFLEN 16
uint8_t keyBuffer[KEYBUFLEN];
uint8_t keyBufPtrIn=0,keyBufPtrOut=0;
uint8_t needToEndIt = 0;

void put_key(uint8_t key)
{
    keyBuffer[keyBufPtrIn++]=key;
    if (keyBufPtrIn>=KEYBUFLEN) keyBufPtrIn=0;
}

uint8_t get_key(uint8_t *key)
{
    if (keyBufPtrOut!=keyBufPtrIn)
    {
        *key = keyBuffer[keyBufPtrOut++];
        if (keyBufPtrOut>=KEYBUFLEN) keyBufPtrOut=0;
        needToEndIt = 1;
        return 1;
    }

    else if (needToEndIt)
    {
        *key = 0;
        needToEndIt = 0;
        return 1;
    }

    *key = 0;
    return 0;
}

uint8_t bufNotEmpty(void)
{
    if ((keyBufPtrOut!=keyBufPtrIn)||(needToEndIt!=0)) return 1;
    return 0;
}

/* ------------------------------------------------------------------------- */

int	main(void)
{
uchar   key, lastKey = 0, keyDidChange = 0;
uchar   idleCounter = 0;

/*uchar bkey; // key from buffer
uchar lastLastKey = 0, lastLastKeySet = 0;
    uint8_t keys, lastKeys=0;
    uint8_t keysQue[16];
    uint8_t keysQueWritePtr=0, keysQueReadPtr=0;*/

	wdt_enable(WDTO_2S);
    hardwareInit();
	odDebugInit();
	usbInit();
	sei();
    DBG1(0x00, 0, 0);
	for(;;){	/* main event loop */
		wdt_reset();
		usbPoll();
		/*keys = keyPressedAtOnce();
		if (lastKeys != keys)
		{
		    uint8_t whichKeys = (lastKeys^keys)&&(~lastKeys);
		    lastKeys = keys;
		    if (whichKeys&0x01) put_key(1);//keysQue[keysQueWritePtr++]=1;
		    //keysQueWritePtr&=0x0F;
		    if (whichKeys&0x02) put_key(2);//keysQue[keysQueWritePtr++]=2;
		    //keysQueWritePtr&=0x0F;
		    if (whichKeys&0x04) put_key(3);//keysQue[keysQueWritePtr++]=3;
		    //keysQueWritePtr&=0x0F;

		    lastKeys=keys;
		}
		if ((keysQueReadPtr!=keysQueWritePtr) && usbInterruptIsReady())
		{
		    buildReport(keysQue[keysQueReadPtr++]);
		    keysQueReadPtr&=0x0F;
		    usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
		}*/

		/*if (lastLastKeySet)
		{
		    lastKey        = lastLastKey;
		    lastLastKeySet = 0;
		    keyDidChange = 1;
		}
		else if (get_key(&bkey))
		{
		    lastLastKey    = lastKey;
		    lastLastKeySet = 1;
		    lastKey = bkey;
		    keyDidChange = 1;
		}
		else
		{*/
		    static uchar lkey = 0;
		    key = keyPressed();
		    if (key!=lkey)
		    {
		        lkey=key;
		        if (key!=0)
		        {
                    put_key(1);
                    put_key(2);
                    put_key(3);
		        }
		    }
            /*key = keyPressed();
            if(lastKey != key)
            {
                lastKey = key;
                keyDidChange = 1;
            }*/
		//}

        if(TIFR & (1<<TOV0)){   // 22 ms timer
            TIFR = 1<<TOV0;
            if(idleRate != 0){
                if(idleCounter > 4){
                    idleCounter -= 5;   // 22 ms in units of 4 ms
                }else{
                    idleCounter = idleRate;
                    keyDidChange = 1;
                }
            }
        }
        if((bufNotEmpty() || keyDidChange) && usbInterruptIsReady()){
            if (bufNotEmpty())
            {
                get_key(&lastKey);
                buildReport(lastKey);
            }
            else
            {
                keyDidChange = 0;
                buildReport(lastKey);
            }
            if (lastKey==0) LED_OFF(); else LED_ON();
            usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        }
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
