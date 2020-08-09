//************************************************************************
//
//16f-based minimalist ballistic chrono
//copyright: dannyelectronics.wordpress.com
//           you are free to modify and redistribute the code 
//           for non-commercial applications
//version history:
//v0.1: 8/6/2020 - initial implementation supporting PIC16/ATMega328/STM32F103
//v0.2: 8/8/2020 - support single capture unit on PIC16
//
//************************************************************************
//
//can be retargetted to similar MCUs:
//hardware requirements:
//1. two banks of 6 consecutive GPIO pins
//2. one 8-bit timer to drive the LED display
//3. one 16-bit timer for time base
//4. one or two Input Capture unit, or GPIO with interrupt capability
//5. at least 2-3KB flash space, and 100 bytes in RAM
//
//
//
//connection:
//
//
//              +---------------+
//              |               |
//              |               |
// SEGE  <------|RA0         RB6|------> DIG1
// SEGD  <------|RA1         RB5|------> SEGA
// SEGDP <------|RA2         RB4|------> SEGF
// SEGC  <------|RA3         RB3|------> DIG2
// SEGG  <------|RA4         RB2|------> DIG3
// DIG4  <------|RA5         RB1|------> SEGB
//              |               |
//              |               |
//              |               |
//              |               |
//              |               |
//              |               |
//              |               |
//      +>------|RC2/CCP1       |
//      |       |               |
//      +<------|RC3 to CCP1    |
//              |               |
//              +---------------+
//
//
//Note: 	Use RC3 to simulate input pulses
//  		connect RC3 to RC2/CCP1
//
//************************************************************************

//using internal oscillator
#include "config.h"							//configuration words
#include "gpio.h"                           //we use gpio functions
#include "delay.h"                          //we use software delays
#include "led4_pins.h"						//we use 4-digit, led display
#include "tmr0.h"							//we use tmr0 to drive the display
#include "tmr1.h"							//we use tmr1 as timebase for the capture pins

#define TK_CNT			30000ul				//number of cycles before the new data is updated

//hardware configuration
//to simulate input pulses
//doesn't hurt if kept in
//about 300bytes in footprint
#define SIG1_PORT		LATC
#define SIG1_DDR		TRISC
#define SIG1_PIN		(1<<3)				//1st pulse - comment out if not needed 

//CCP1 pins
#define P1_DDR			TRISC				//1st pulse
#define P1_PIN			(1<<2)				//CCP1 on RC2
#define DISTmm			123					//distance between the sensors, in mm (max length = 4292mm)
#define TICK_US			4					//number of TIMER1 ticks per us
#define T2SPD(tick)		T2SPDm(tick)		//convert tick to speed. can be mapped to T2SPDm() or T2SPDft()
//end hardware configuration

//global defines
//need to pay attention to overflow when DISTmm is large
//need to ensure that the product of the first two not exceed 2^32
#define T2SPDm(tick)	((DISTmm) * (1000000ul/20) / (tick) * TICK_US * 20)	//convert tick to speed, in 0.001 mm/s
#define T2SPDft(tick)	((DISTmm) * (3280840ul/40) / (tick) * TICK_US * 40)	//convert tick to speed, in 0.001 ft/s
//1 meter = 3.28084ft

//global variables
extern unsigned char lRAM[4];				//4 digit display buffer
volatile unsigned short t0cnt;				//elapsed ticks between pulses
unsigned long spdx1k;						//speed x 1000, in m/s or ft/s, depending on the mapping of T2SPD()
volatile char chrono_state;					//0->no new data, 1->1st edge has arrived, 2->2nd edge has arrived = new data available

//isr
void interrupt isr(void) {
	//unsigned char tmp_msb=CCPR1H, tmp_lsb=CCPR1L;
	//static unsigned short tmp1;
	unsigned short tmp;
	
	//execution ordered based on priority
	//read tmr1 captured value
	//tmp = (CCPR1H << 8) | CCPR1L;
	if (CCP1IF) {							//read the 1st pulse
		tmp = (CCPR1H << 8) | CCPR1L; 
		CCP1IF = 0;
		//switch (chrono_state) {
		//	case 0: t0cnt = tmp; chrono_state = 1; break;
		//	case 1: t0cnt = tmp - t0cnt; chrono_state = 2; break;
		//}	
		if (chrono_state == 0) {t0cnt = tmp; chrono_state = 1;}
		else {t0cnt = tmp - t0cnt; chrono_state = 2;}
	}

	//led display routine
	if (TMR0IF) tmr0_isr();					//tmr0 isr - for led display
}

//initialize the chrono
//tmr1 as time base
//ccp1 as leading pulse, ccp2 as falling pulse
//capture on falling edge
void chrono_init(void) {
	//initialize status
	chrono_state=0;							//chrono state. 0-> no data; 1->leading pulse detected; 2->falling edge detected == new data available
	t0cnt=0;								//reset tick counter
	
	//configure CCP1
	IO_IN(P1_DDR, P1_PIN);					//as input
	CCP1CON = 0b0100;						//0b0100->on every falling edge, 0b0101->on every rising edge
	CCP1IF = 0;								//clear the flag
	CCP1IE = 1;								//1->enable the interrupt
	PEIE = 1;								//1->enable periopheral interrupt, 0->disable it

	//initialize the time base for capture
	//free-running 16-bit timer
	tmr1_init(TMR1_PS1x, 0);				//initialize tmr1: clock source = Fosc/4, period = 0xffff
	
}

int main(void) {
	unsigned long loop_cnt=0, tick_disp=0;
	unsigned long tmp;
	char dp;
	//char tick_new=0;
	
	mcu_init();							    //initialize the mcu -> 16Mhz IRC
	delay_ms(50);							//for debugging
	
	//initialize the 4-digit led display
	led_init();								//initialize the led display
	//initialize the display buffer
	lRAM[0]=lRAM[1]=lRAM[2]=lRAM[3]=0xff;	//display all segments	
	//initialize the display
	tmr0_init(TMR0_PS_64x);					//initialize tmr0 -> overflows at 1/4us * 256 * 128 = 8ms/digit = 32ms per frame -> 30Hz
	tmr0_act(led_display);					//install the display handler

	//initialize the chrono
	chrono_init();

#if defined(SIG1_PIN)
	//initialize signal generator
	IO_CLR(SIG1_PORT, SIG1_PIN); IO_OUT(SIG1_DDR, SIG1_PIN);
	//IO_CLR(SIG2_PORT, SIG2_PIN); IO_OUT(SIG2_DDR, SIG2_PIN);
#endif
		
	ei();									//enable interrupt
	while (1) {
		
#if defined(SIG1_PIN)
		//*****************NOTE*********************
		//****for simulation / code testing only****
		//after a certain passage, generate two pulses
		loop_cnt+=1;						//increment loop count
		if (loop_cnt - tick_disp > TK_CNT) {
			tick_disp+=TK_CNT;				//update time
			//generate the first pulse
			IO_SET(SIG1_PORT, SIG1_PIN); NOP4(); IO_CLR(SIG1_PORT, SIG1_PIN);
			NOP64();						//time elapsed between the two pulses
			NOP64();						//time elapsed between the two pulses
			NOP64();						//time elapsed between the two pulses
			NOP64();						//time elapsed between the two pulses
			//generate the 2nd pulse
			IO_SET(SIG1_PORT, SIG1_PIN); NOP4(); IO_CLR(SIG1_PORT, SIG1_PIN);
		}
		//******************end**********************
#endif
		//detecting the 2nd pulse
		//if digit 4's decimal point is on, need to reset the chip
		if (chrono_state == 1) lRAM[3]|=0x80;	//1->turn on the decimal point for digit 4, indicating that the first pulse has arrived
		
		//update display if new data has arrived		
		if (chrono_state == 2) {				//2->new data is available
			chrono_state = 0;					//reset tick_new -> 0=no new data
			
			//convert ticks elapsed to speed
			spdx1k= T2SPD(t0cnt);				//convert ticks to speed
			//spdx1k= t0cnt;					//to just display t0cnt for debugging only
			
			//form the display buffer
			tmp = spdx1k;
			     if (tmp >10000000ul-1) tmp = 10000000ul-1;					//max display is "9999."
			     if (tmp > 1000000ul-1) {tmp = (tmp + 500) / 1000; dp=3;}	//1000 - 9999 -> display "xxxx.", rounding up
			else if (tmp >  100000ul-1) {tmp = (tmp +  50) /  100; dp=2;}	//100. - 999. -> display "xxx.x", rounding up
			else if (tmp >   10000ul-1) {tmp = (tmp +   5) /   10; dp=1;}	//10.0 - 99.9 -> display "xx.xx", rounding up
			else                        {tmp = (tmp +   0) /    1; dp=0;}	//1.00 - 9.00 -> display "x.xxx", rounding up
			lRAM[3]=ledfont_num[tmp % 10]; tmp /=10;
			lRAM[2]=ledfont_num[tmp % 10]; tmp /=10;
			lRAM[1]=ledfont_num[tmp % 10]; tmp /=10;
			lRAM[0]=ledfont_num[tmp % 10]; tmp /=10;
			switch (dp) {
				case 0:
				case 1:
				case 2: lRAM[dp]|=0x80; break;	//add decimal point
				//case 3:						//do nothing - DP on the last led used as error indicator
			}
		}
	}
}
