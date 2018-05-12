/*******************************************************************************

	Copyright (c) 2008 Wizign Co., Ltd.
    All Rights Reserved.

	main.c: Main program of the radio clock project

	Date: Jun 09, 2008

	Author: YenHung Chen

	E-mail: yhchen@wizign.com

	Revision:
	---------- -----------------------------------------------------------------
	2008/06/09 Created by YenHung Chen, Concept of CME6005 C-MAX

 ******************************************************************************/
/*******************************************************************************

<Concept of CME6005 C-MAX>

Module: CMMR-6P-60
Frequency: 60kHz
Transmitting power: 50kW


<>Timer center in the world

Timer Center    Timer code  Frequency(Hz)
U.S.A           WWVB        60K
German          DCF77       77.5K
U.K.            MSF         60K
Japan           JJY40       40K
                JJY60       60K
China           BPC         68.5K
Switzerland     HBG         75K


<>Station source

Station: Hagane-yama
Location: Fuji vil., saga Pref.
Frequency: 60kHz
Geographical coordinates: 33'28"N, 130'11"E
Transmitting power: 50kW
Time of transmission: permanent

Time frame 1 minute (Index count 1 second)
  ON
 +---+   +-
 |   |   |
-+   +---+
      OFF

1 : 0.5s
0 : 0.8s
P : 0.2s (Position identifier markers P0..P5)

A time frame contains BCD-coded information of minutes, hours, days, weeks and year.

  0               5          10             15         20               25             30         35
  |               |          |              |          |                |              |          |
P0|FRM|40|20|10|xx|8|4|2|1|P1|xx|xx|20|10|xx|8|4|2|1|P2|xx|xx|200|100|xx|80|40|20|10|P3|8|4|2|1|xx|
      |<---  minutes  --->|        |<--- hours  --->|        |<---          days           --->|

35                40              45         50            55         60
|                 |               |          |             |          |
|xx|Pa1|Pa2|SU1|P4|SU2|80|40|20|10|8|4|2|1|P5|4|2|1|LS1|LS2|0|0|0|0|P0|
                      |<---    year   --->|  |<- ->|<-   ->|

frame: 1
minute(7): 2-4,6-9
hour(6): 13-14,16-19
day(10): 23-24,26-29,31-34
year(8): 42-49


<>Logical flow

while(loop){
    read bit
    if(start of frame){
        read time
    }
}


<>Define IO

P1.0 : TCO (only need this pin)
P1.1 : TCON
P1.2 : HLD


 ******************************************************************************/

//#include <mcs51/at89x52.h>
#include <atmel/at89x52.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Setup MCU's clock

#define OSCILLATOR 12000000

// timer 0 used for system clock
//#define TIMER0_RELOAD_VALUE OSCILLATOR/12/500     // 0.5 millisecond for 12Mhz
#define TIMER0_RELOAD_VALUE (OSCILLATOR/12/1000)  // 1 millisecond for 12Mhz
#define TIMER_MAX 65536

static long data milliSeconds;

void ClockIrqHandler (void) interrupt 1 using 3
{
	TL0 = (TIMER_MAX - TIMER0_RELOAD_VALUE) % 0xff;
  	TH0 = (TIMER_MAX - TIMER0_RELOAD_VALUE) / 0xff;
  	milliSeconds++;
}

// we can't just use milliSeconds
unsigned long TickClock(void)
{
  	unsigned long ms;
  	ET0=0;
  	ms=milliSeconds;
  	ET0=1;
  	return ms;

  	//return milliSeconds;
}

// start clock
unsigned char StartClock(void)
{
  	// initialize timer0 for system clock
  	TR0=0; // stop timer 0
  	TMOD =(TMOD&0xf0)|0x01; // T0=16bit timer

  	// timeout is xtal/12
  	TL0 = (TIMER_MAX - TIMER0_RELOAD_VALUE) % 0xff;
  	TH0 = (TIMER_MAX - TIMER0_RELOAD_VALUE) / 0xff;
  	milliSeconds=0; // reset system time
  	TR0=1; // start timer 0
  	ET0=1; // enable timer 0 interrupt

  	EA=1; // enable global interrupt

  	return 0;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Read radio clock signal

/*
  time information
 */
typedef struct tag_TimeInfo {
    int yr;
    int mon;
    int day;
    int hr;
    int min;
    int sec;
} TimeInfo;
/*
  Holds all the state information about a clock receiver
frame: 1
minute(7): 2-4,6-9
hour(6): 13-14,16-19
day(10): 23-24,26-29,31-34
year(8): 42-49
 */
#define MINSTART  2
#define MIN40     0     // 2
#define MIN20     1     // 3
#define MIN10     2     // 4
#define MIN08     3     // 6
#define MIN04     4     // 7
#define MIN02     5     // 8
#define MIN01     6     // 9
#define HRSTART  13
#define HR20      0     // 13
#define HR10      1     // 14
#define HR08      2     // 16
#define HR04      3     // 17
#define HR02      4     // 18
#define HR01      5     // 19
#define DAYSTART  23
#define DAY200     0
#define DAY100     1
#define DAY080     2
#define DAY040     3
#define DAY020     4
#define DAY010     5
#define DAY008     6
#define DAY004     7
#define DAY002     8
#define DAY001     9
#define YRSTART   42
#define YR80       0
#define YR40       1
#define YR20       2
#define YR10       3
#define YR08       4
#define YR04       5
#define YR02       6
#define YR01       7

typedef union tag_TimeBits {
    char bits[31];
    struct time_bits {
        char min[7];
        char hr[6];
        char day[10];
        char yr[8];
    } tb;
} TimeBits;

typedef struct tag_ClockInfo {
	int count;      // bit position
	int status;     // success or failed
	int error;      // none or error code
	char bframe;    // start of frame
	TimeBits tbits;
    TimeInfo tnow;  // current time
    TimeInfo tlast; // last recorded time
	char carea[16]; // code area
} ClockInfo;

/*
  Define IO function and pin map
P1_0 : radio clock signal
 */
#define RCSignal P1_0

typedef enum tag_SIGNAL_TYPE {
    SIGNAL_NONE,
    SIGNAL_LOW,
    SIGNAL_HIGH,
    SIGNAL_OFF,
    SIGNAL_ON,
    SIGNAL_GO,
    MARKER_START,
    MARKER_P0,
    MARKER_P1,
    MARKER_P2,
    MARKER_P3,
    MARKER_P4,
    MARKER_P5
} SIGNAL_TYPE;

ClockInfo radioClock = {
    0, 0, 0, 0,
    {{0}},
    {0,0,0,0,0,0},
    {0,0,0,0,0,0},
    "JJY60"
};

SIGNAL_TYPE SearchFrame(SIGNAL_TYPE presg,ClockInfo *pclock)
{
    SIGNAL_TYPE sg = SIGNAL_NONE;
    static unsigned long oldtime = 0;
    unsigned long newtime = 0;
    unsigned long deltatime = 0;

    if(RCSignal == 1) { // signal high
        if(presg == MARKER_START){  // looking for the frame
            newtime = TickClock();
            deltatime = newtime - oldtime;

            if((deltatime < 1750) || (deltatime > 1850)){
                sg = SIGNAL_NONE;
                oldtime = newtime;
            }else{  // 1800ms
                pclock->bframe = 1;
                pclock->count = 1;
                sg = MARKER_P0;
                oldtime = newtime;
            }
        }else{
            sg = SIGNAL_HIGH;
            oldtime = newtime = TickClock();
        }
    }else{  // signal low
        newtime = TickClock();
        deltatime = newtime - oldtime;

        if(presg == MARKER_START){
            sg = MARKER_START;
        }else if(presg == SIGNAL_HIGH){
            if((deltatime > 150)&&(deltatime < 250)){   // 200ms, marker Pn
                sg = MARKER_START;
            }else{
                sg = SIGNAL_LOW;
            }
            oldtime = newtime;
        }else{  // signal low
            sg = SIGNAL_LOW;
        }
    }
    return sg;
}

SIGNAL_TYPE ReadSignal(SIGNAL_TYPE presg,ClockInfo *pclock)
{
    SIGNAL_TYPE sg = SIGNAL_NONE;
    static unsigned long oldtime = 0;
    unsigned long newtime = 0;
    unsigned long deltatime = 0;

    if(RCSignal == 1) { // signal high
        sg = SIGNAL_HIGH;
        oldtime = newtime = TickClock();
    }else{  // signal low
        newtime = TickClock();
        deltatime = oldtime - newtime;

        if(presg == SIGNAL_HIGH){
            pclock->count++;

            if(deltatime < 250){ // 200ms, marker
                switch(pclock->count){
                case 10:
                    sg = MARKER_P1;
                    break;
                case 20:
                    sg = MARKER_P2;
                    break;
                case 30:
                    sg = MARKER_P3;
                    break;
                case 40:
                    sg = MARKER_P4;
                    break;
                case 50:
                    sg = MARKER_P5;
                    break;
                default:
                    sg = SIGNAL_NONE;
                    break;
                }
            }else if((deltatime > 450) && (deltatime < 550)) {    // 500ms, on
                sg = SIGNAL_ON;
            }else if((deltatime > 750) && (deltatime < 850)) {    // 800ms, off
                sg = SIGNAL_OFF;
            }else{
                sg = SIGNAL_NONE;
            }
        }else{  // unknown
            sg = SIGNAL_NONE;
            oldtime = newtime;
        }
    }
    return sg;
}

void CalculateTime(char sg, ClockInfo *pclock)
{
    switch(pclock->count){
    case 2:
        pclock->tbits.tb.min[MIN40] = sg;
        break;
    case 3:
        pclock->tbits.tb.min[MIN20] = sg;
        break;
    case 4:
        pclock->tbits.tb.min[MIN10] = sg;
        break;
    case 6:
        pclock->tbits.tb.min[MIN08] = sg;
        break;
    case 7:
        pclock->tbits.tb.min[MIN04] = sg;
        break;
    case 8:
        pclock->tbits.tb.min[MIN02] = sg;
        break;
    case 9:
        pclock->tbits.tb.min[MIN01] = sg;
        break;
    case 13:
        pclock->tbits.tb.hr[HR20] = sg;
        break;
    case 14:
        pclock->tbits.tb.hr[HR10] = sg;
        break;
    case 16:
        pclock->tbits.tb.hr[HR08] = sg;
        break;
    case 17:
        pclock->tbits.tb.hr[HR04] = sg;
        break;
    case 18:
        pclock->tbits.tb.hr[HR02] = sg;
        break;
    case 19:
        pclock->tbits.tb.hr[HR01] = sg;
        break;
    case 23:
        pclock->tbits.tb.day[DAY200] = sg;
        break;
    case 24:
        pclock->tbits.tb.day[DAY100] = sg;
        break;
    case 26:
        pclock->tbits.tb.day[DAY080] = sg;
        break;
    case 27:
        pclock->tbits.tb.day[DAY040] = sg;
        break;
    case 28:
        pclock->tbits.tb.day[DAY020] = sg;
        break;
    case 29:
        pclock->tbits.tb.day[DAY010] = sg;
        break;
    case 31:
        pclock->tbits.tb.day[DAY008] = sg;
        break;
    case 32:
        pclock->tbits.tb.day[DAY004] = sg;
        break;
    case 33:
        pclock->tbits.tb.day[DAY002] = sg;
        break;
    case 34:
        pclock->tbits.tb.day[DAY001] = sg;
        break;
    case 42:
        pclock->tbits.tb.yr[YR80] = sg;
        break;
    case 43:
        pclock->tbits.tb.yr[YR40] = sg;
        break;
    case 44:
        pclock->tbits.tb.yr[YR20] = sg;
        break;
    case 45:
        pclock->tbits.tb.yr[YR10] = sg;
        break;
    case 46:
        pclock->tbits.tb.yr[YR08] = sg;
        break;
    case 47:
        pclock->tbits.tb.yr[YR04] = sg;
        break;
    case 48:
        pclock->tbits.tb.yr[YR02] = sg;
        break;
    case 49:
        pclock->tbits.tb.yr[YR01] = sg;
        break;
    default:    // no action
        break;
    }
}

void ReadRadioClock(ClockInfo *pclock)
{
    /* Declare parameters */
    static SIGNAL_TYPE sigtype = SIGNAL_NONE;

    if(pclock->bframe == 0){
        sigtype = SearchFrame(sigtype, pclock);
    }else{   // radioClock.bframe == 1, To decode the time frame
        sigtype = ReadSignal(sigtype, pclock);
        if(pclock->count < 60){
            if((pclock->count == 50) && (sigtype == MARKER_P5)){
                pclock->bframe = 0;
                pclock->count = 0;
                sigtype = SIGNAL_NONE;
            }
            if(sigtype == SIGNAL_ON){
                CalculateTime(1, pclock);
            }else if(sigtype == SIGNAL_OFF){
                CalculateTime(0, pclock);
            }else{  // unknown condition
            }
        }else{  // count >= 60, means error
            pclock->bframe = 0;
            pclock->count = 0;
            sigtype = SIGNAL_NONE;
        }
    }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// main program

void main(void)
{
    unsigned long oldtm;
    unsigned long newtm;
    unsigned long deltatm;
    //int count;

    /* Initialize all parameters */
    RCSignal = 0;

    // start timer0
    StartClock();

    newtm = oldtm = TickClock();

    /* do loop */
    while(1){
        newtm = TickClock();
        deltatm = newtm - oldtm;

        if(deltatm > 1000){
        //if(milliSeconds > 1000){
            P2 = (P2==0) ? 0xff : 0;
        //    milliSeconds = 0;
            oldtm = newtm;
        }
        ReadRadioClock(&radioClock);
        //count = 100;
        //while(count--)
        //    ;
    }
}


//#define MCU_8051
// #define MCU_PIC

/*
//#ifdef MCU_PIC

#include <pic/pic16f877.h>

// Configurations
    typedef unsigned int config;
    config at 0x2007 __CONFIG = _HS_OSC & _PWRTE_ON & _BODEN_OFF & _WDT_OFF & _LVP_OFF;

//#endif

// Main body
void main() {

    // Initializing ports
    PORTA = 0;
    PORTB = 0;

    // Set RA4 as input and RB3-RB0 as output
    TRISA |= 0x10;
    TRISB &= 0xF0;

    // Set value 0x0A to PORTB
    PORTB = 0x0A;

    // If button is pressed, toggle PORTB
    while(1) {
        if(RA4 != 0)
            PORTB = ~PORTB;
    }
}
*/
