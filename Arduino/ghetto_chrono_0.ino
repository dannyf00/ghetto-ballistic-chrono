//ghetto ballastic chrono on Arduino Pro Mini / ATMega328p
//ghetto_chrono_0.ino
//************************************************************************
//
//Arduino-based minimalist ballistic chrono
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
// SEGE  <------|2         17/A3|------> DIG1
// SEGD  <------|3         16/A2|------> SEGA
// SEGDP <------|4         15/A1|------> SEGF
// SEGC  <------|5         14/A0|------> DIG2
// SEGG  <------|6            13|------> DIG3
// DIG4  <------|7            12|------> SEGB
//              |               |
//              |               |
//              |               |
//      +>------|8/ICP1         |
//      |       |               |
//      +<------|9              |
//              |               |
//              +---------------+
//
// Pin 9 simulates a pulse train for Pin 8/ICP1 to test the program
//

//hardware configuration
#define SEGE	2
#define SEGD	3
#define SEGDP	4
#define SEGC	5
#define SEGG	6
#define DIG4	7

#define DIG1	17
#define SEGA	16
#define SEGF	15
#define DIG2	14
#define DIG3	13
#define SEGB	12

#define DISPLAY_IS_CA						//uncomment if display is common cathode
//to simulate input pulses
//comment out if not needed
//doesn't hurt to keep in
#define SIG1_PIN		9					//pulse generator

//CCP1 pins
#define P1_PIN			8					//CCP1 on RC2
#define DISTmm			123					//distance between the sensors, in mm (max length = 4292mm)
#define TICK_US			16					//number of TIMER1 ticks per us
#define T2SPD(tick)		T2SPDm(tick)		//convert tick to speed. can be mapped to T2SPDm() or T2SPDft()
#define TK_CNT			30000ul				//number of cycles before the new data is updated
//end hardware configuration

//global defines
//mapping GPIO macros to Arduino
#define IO_SET(port, pin)		digitalWrite(pin, HIGH)
#define IO_CLR(port, pin)		digitalWrite(pin, LOW)
#define IO_OUT(ddr, pin)		pinMode(pin, OUTPUT)
#define IO_IN(ddr, pin)			pinMode(pin, INPUT)

#if defined(DISPLAY_IS_CA)		//DISPLAY_MODE==CA	//for common anode displays
//digit control - active high (Common Anode) or active low (Common Cathode)
#define DIG_ON(port, pins)		IO_SET(port, pins)			//turn on a digit
#define DIG_OFF(port, pins)	 	IO_CLR(port, pins)			//turn off a digit

//segment control - active low (Common Anode) or active high (Common Cathode)
#define SEG_ON(port, pins)		IO_CLR(port, pins)			//turn on a segment
#define SEG_OFF(port, pins)	 	IO_SET(port, pins)			//turn off a segment

#else				 			//for common cathode displays
//digit control - active high (Common anode) or active low (Common Cathode)
#define DIG_ON(port, pins)		IO_CLR(port, pins)			//turn on a digit
#define DIG_OFF(port, pins)	 	IO_SET(port, pins)			//turn off a digit

//segment control - active low (Common anode) or active high (Common Cathode)
#define SEG_ON(port, pins)		IO_SET(port, pins)			//turn on a segment
#define SEG_OFF(port, pins)	 	IO_CLR(port, pins)			//turn off a segment
#endif

#define NOP()				asm("nop")		//nop
#define NOP2()				{NOP(); NOP();}
#define NOP4()				{NOP2(); NOP2();}
#define NOP8()				{NOP4(); NOP4();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP24()				{NOP16(); NOP8();}
#define NOP32()				{NOP16(); NOP16();}
#define NOP40()				{NOP32(); NOP8();}
#define NOP64()				{NOP32(); NOP32();}

//global variables
int led = 13;
unsigned char lRAM[4];						//led display buffer
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

//led font.
//SEGDP  = 0x80
//SEGG   = 0x40
//SEGF   = 0x20
//SEGE   = 0x10
//SEGD   = 0x08
//SEGC   = 0x04
//SEGB   = 0x02
//SEGA   = 0x01
//led font for numerical display '0'..'9''a'..'f', active high
const unsigned char ledfont_num[]={		//led font, for common anode
	0x3f,								//'0'
	0x06,								//'1'
	0x5b,								//'2'
	0x4f,								//'3'
	0x66,								//'4'
	0x6d,								//'5'
	0x7d,								//'6'
	0x07,								//'7'
	0x7f,								//'8'
	0x6f,								//'9'
	0x5f,								//'a'
	0x7c,								//'b'
	0x58,								//'c'
	0x5e,								//'d'
	0x79,								//'e'
	0x71,								//'f'
	0x00,								//' ' blank
	//with decimal point. + 17 offset
};
//led font for alphabetic display 'a'..'z'
const unsigned char ledfont_alpha[]={		//led font, for common anode
	0x5f,								//'a'
	0x7c,								//'b'
	0x58,								//'c'
	0x5e,								//'d'
	0x79,								//'e'
	0x71,								//'f'
	0x6f,								//'g'
	0x07,								//'h'
	0x74,								//'i'
	0x0e,								//'j'
	0x00,								//'k'
	0x38,								//'l'
	0x00,								//'m'
	0x54,								//'n'
	0x5c,								//'o'
	0x73,								//'p'
    0x67,								//'q'
    0x77,								//'r'
    0x6d,								//'s'
    0x00,								//'t'
    0x1c,								//'u'
    0x00,								//'v'
    0x00,								//'w'
    0x00,								//'x'
    0x6e,								//'y'
    0x00,								//'z'
	0x00,								//' ' blank
};

//initialize the pins
void led_init(void) {
	//turn off the digits send set pins to output
	DIG_OFF(DIG1_PORT, DIG1); IO_OUT(DIG1_DDR, DIG1);
	DIG_OFF(DIG2_PORT, DIG2); IO_OUT(DIG2_DDR, DIG2);
	DIG_OFF(DIG3_PORT, DIG3); IO_OUT(DIG3_DDR, DIG3);
	DIG_OFF(DIG4_PORT, DIG4); IO_OUT(DIG4_DDR, DIG4);

	//turn off the segments
	SEG_OFF(SEGA_PORT, SEGA); IO_OUT(SEGA_DDR, SEGA);
	SEG_OFF(SEGB_PORT, SEGB); IO_OUT(SEGB_DDR, SEGB);
	SEG_OFF(SEGC_PORT, SEGC); IO_OUT(SEGC_DDR, SEGC);
	SEG_OFF(SEGD_PORT, SEGD); IO_OUT(SEGD_DDR, SEGD);
	SEG_OFF(SEGE_PORT, SEGE); IO_OUT(SEGE_DDR, SEGE);
	SEG_OFF(SEGF_PORT, SEGF); IO_OUT(SEGF_DDR, SEGF);
	SEG_OFF(SEGG_PORT, SEGG); IO_OUT(SEGG_DDR, SEGG);
	SEG_OFF(SEGDP_PORT, SEGDP); IO_OUT(SEGDP_DDR, SEGDP);

	//set all pins to output
	//put your code here
}

//display the ledram
void led_display(void) {
	static unsigned char dig=0;				//current digit
	unsigned char tmp;

	//turn off the digits
	DIG_OFF(DIG1_PORT, DIG1); 
	DIG_OFF(DIG2_PORT, DIG2); 
	DIG_OFF(DIG3_PORT, DIG3); 
	DIG_OFF(DIG4_PORT, DIG4); 

	//if user filled lRAM with numbers only
	//tmp=ledfont_num[lRAM[dig]];			//retrieve font / segment info from the display buffer
	//if user filled lRAM with segment information
	tmp=lRAM[dig];							//retrieve font / segment info from the display buffer
	//turn on/off the segments
	if (tmp & 0x01) SEG_ON(SEGA_PORT, SEGA); else SEG_OFF(SEGA_PORT, SEGA);
	if (tmp & 0x02) SEG_ON(SEGB_PORT, SEGB); else SEG_OFF(SEGB_PORT, SEGB);
	if (tmp & 0x04) SEG_ON(SEGC_PORT, SEGC); else SEG_OFF(SEGC_PORT, SEGC);
	if (tmp & 0x08) SEG_ON(SEGD_PORT, SEGD); else SEG_OFF(SEGD_PORT, SEGD);
	if (tmp & 0x10) SEG_ON(SEGE_PORT, SEGE); else SEG_OFF(SEGE_PORT, SEGE);
	if (tmp & 0x20) SEG_ON(SEGF_PORT, SEGF); else SEG_OFF(SEGF_PORT, SEGF);
	if (tmp & 0x40) SEG_ON(SEGG_PORT, SEGG); else SEG_OFF(SEGG_PORT, SEGG);
	if (tmp & 0x80) SEG_ON(SEGDP_PORT, SEGDP); else SEG_OFF(SEGDP_PORT, SEGDP);

	//turn on the digit and advance to the next digit
	switch (dig) {
		case 0: DIG_ON(DIG1_PORT, DIG1); dig=1; break;
		case 1: DIG_ON(DIG2_PORT, DIG2); dig=2; break;
		case 2: DIG_ON(DIG3_PORT, DIG3); dig=3; break;
		case 3: DIG_ON(DIG4_PORT, DIG4); dig=0; break;
	}
}

//prescaler
#define TMR2_PSNOCLK		0x00
#define TMR2_PS1x			0x01
#define TMR2_PS8x			0x02
#define TMR2_PS32x			0x03
#define TMR2_PS64x			0x04
#define TMR2_PS128x			0x05
#define TMR2_PS256x			0x06
#define TMR2_PS1024x		0x07
#define TMR2_PSMASK			0x07

//tmr2 used to drive the led display
void empty_handler(void) {
}

static void (* /*_tmr1*/_isrptr)(void)=empty_handler;	//tmr1_ptr pointing to empty_handler by default

//tmr2 isr - ctc mode
ISR(TIMER2_COMPA_vect) {
	/*_tmr1*/_isrptr();						//execute the handler
}

//reset the tmr
//ctc mode: wgm2..0 = 0b010
void tmr2_init(uint8_t prescaler, uint8_t pr) {
	//initialize the handler
	_isrptr = empty_handler;

	//initialize the timer
	TCCR2B =	TCCR2B & (~TMR2_PSMASK);	//turn off tmr1
	TCCR2A |=	(0<<COM2A1) | (0<<COM2A0) |	//output compare a pins normal operation
				(0<<COM2B1) | (0<<COM2B0) |	//output compare b pins normal operation
				//(0<<COM1C1) | (0<<COM1C0) |	//output compare c pins normal operation
				(1<<WGM21) | (0<<WGM20)		//wgm2..0 = 0b010 -> normal mode
				;
	TCCR2B = 	(TCCR2B & ~(1<<WGM21)) |
				(0<<WGM21);
	TCNT2 = 0;								//reset the timer / counter
	OCR2A = pr - 1;
	TIFR2 |= (0<<TOV2) | (1<<OCF2A) |  (0<<OCF2B);						//clear the flag by writing '1' to it
	TIMSK2 =		//(0<<TICIE1) |				//input capture isr: disabled
				//(0<<OCIE1C) |				//output compare isr for ch a: disabled
				(0<<OCIE2B) |				//output compare isr for ch b: disabled
				(0<<OCIE2A) |				//output compare isr for ch c: disabled
				(0<<TOIE2)					//tmr overflow interrupt: disabled
				;
	TCCR2B |=	(prescaler & TMR2_PSMASK)	//prescaler, per the header file
				;
	//now timer1 is running
}

void tmr2_act(void (*isr_ptr)(void)) {
	/*_tmr1*/_isrptr=isr_ptr;				//reassign tmr1 isr ptr
	TIFR2 |= (0<<TOV2) | (1<<OCF2A) | (0<<OCF2B);						//clear the flag by writing '1' to it
	TIMSK2 |=	(0<<TOIE2) | (0<<OCIE2B) | (1<<OCIE2A)					//tmr overflow interrupt: enabled
				;
}

#define TMR1_NOCLK			0x00		//cs210=no clock selected
#define TMR1_PS1x			0x01		//clk/1
#define TMR1_PS8x			0x02		//clk/8
#define TMR1_PS64x			0x03		//clk/64
#define TMR1_PS256x			0x04		//clk/256
#define TMR1_PS1024x		0x05		//clk/1024
#define TMR1_EXTN			0x06		//external clock on Tn pin, negative transistion
#define TMR1_EXTP			0x07		//external clock on Tn pin, positive transistion
#define TMR1_PSMASK			0x07
//for portability
//tmr1
#define TCCRxA			TCCR1A
#define TCCRxB			TCCR1B
#define OCRxA			OCR1A
#define OCRxB			OCR1B
#define TCNTx			TCNT1
#define TIFRx			TIFR1
#define TIMSKx			TIMSK1
#define COMxA1			COM1A1
#define COMxA0			COM1A0
#define COMxB1			COM1B1
#define COMxB0			COM1B0
#define WGMx3			WGM13
#define WGMx2			WGM12
#define WGMx1			WGM11
#define WGMx0			WGM10
#define OCFxA			OCF1A
#define OCFxB			OCF1B
#define TOVx			TOV1
#define OCIExA			OCIE1A
#define OCIExB			OCIE1B
#define TOIEx			TOIE1
#define TMRx_PSMASK		TMR1_PSMASK

//global defines

//global variables

//empty handler
//static void /*_tmr1_*/empty_handler(void) {
	//default tmr handler
//}

//static void (* /*_tmr1*/_isrptr)(void)=empty_handler;	//tmr1_ptr pointing to empty_handler by default

//tmr1 isr
//ISR(TIMER1_COMPA_vect) {
//	/*_tmr1*/_isrptr();						//execute the handler
//}

//reset the tmr
//free running, capture mode
void tmr1_init(uint8_t prescaler) {
	///*_tmr1*/_isrptr=/*_tmr1_*/empty_handler;			//reset isr ptr

	//set up the timer
	TCCRxB =	TCCRxB & (~TMRx_PSMASK);	//turn off tmr1
	TCCRxA =	(0<<COMxA1) | (0<<COMxA0) |	//output compare a pins normal operation
				(0<<COMxB1) | (0<<COMxB0) |	//output compare b pins normal operation
				//(0<<COM1C1) | (0<<COM1C0) |	//output compare c pins normal operation
				(0<<WGMx1) | (0<<WGMx0)		//wgm13..0 = 0b0000 -> normal mode
				;
	TCCRxB =	(0<<ICNC1) |				//0->no input noise cancelling
				(0<<ICES1) |				//0->falling edge, 1->rising edge
				(TCCRxB & ~((1<<WGMx3) | (1<<WGMx2))) |	//clear wgm13..2
				(0<<WGMx3) | (0<<WGMx2);	//wgm13.0=0b0000 -> normal mode
	//TCCR1C =	(0<<FOC1A) |				//forced output on ch a disabled
	//			(0<<FOC1B) |				//forced output on ch b disabled
	//			(0<<FOC1C)					//forced output on ch c disabled
	//			;
	//OCRxA = pr - 1;
	TCNTx = 0;								//reset the timer / counter
	TIFRx |= (1<<ICF1);						//1->clear ICF1 flag by writing 1 to it
	TIMSKx =	(1<<ICIE1) |				//input capture isr: 0->disabled, 1->enabled
				//(0<<OCIE1C) |				//output compare isr for ch a: 0->disabled
				(0<<OCIExB) |				//output compare isr for ch b: 0->disabled
				(0<<OCIExA) |				//output compare isr for ch c: 0->disabled
				(0<<TOIEx)					//tmr overflow interrupt: disabled
				;
	TCCRxB |=	(prescaler & TMRx_PSMASK)	//prescaler, per the header file
				;
	//now timer1 is running
}

//icp1 interrupt here
ISR(TIMER1_CAPT_vect) {
	uint16_t tmp;

	//flag is cleared automatically
	tmp = ICR1;							//save the captured data first
	if (chrono_state == 0) {t0cnt = tmp; chrono_state = 1;}
	else {t0cnt = tmp - t0cnt; chrono_state = 2;}
}

//set up the chrono
//initialize the chrono
//tmr1 as time base
//ccp1 as leading pulse, ccp2 as falling pulse
//capture on falling edge
void chrono_init(void) {
	//initialize status
	chrono_state=0;						//chrono state. 0-> no data; 1->leading pulse detected; 2->falling edge detected == new data available
	t0cnt=0;							//reset tick counter
	
	//configure CCP1
	IO_IN(P1_DDR, P1_PIN);				//as input
	//CCP1CON = 0b0101;					//0b0100->on every falling edge, 0b0101->on every rising edge
	//CCP1IF = 0;						//clear the flag
	//CCP1IE = 1;						//1->enable the interrupt
	
	//configure CCP2
	//IO_IN(P2_DDR, P2_PIN);			//as input
	//CCP2CON = 0b0101;					//0b0100->on every falling edge, 0b0101->on every rising edge
	//CCP2IF = 0;						//clear the flag
	//CCP2IE = 1;						//1->enable the interrupt
	//PEIE = 1;							//enable periopheral interrupt

	//initialize the time base for capture
	tmr1_init(TMR1_PS1x);				//initialize tmr1: clock source = Fosc/4, period = 0xffff
}

void setup() {
  	// put your setup code here, to run once:
  	//pinMode(led, OUTPUT);

  	//initialize the display
  	led_init();

	//initialize the display buffer
	lRAM[0]=lRAM[1]=lRAM[2]=lRAM[3]=0xff;	//display all segments	
	//initialize the display
  	tmr2_init(TMR2_PS128x, 0);			//128x prescaler, 0xff period. 1/16us * 256 * 128 = 2ms/digit, or 8ms/frame -> 125Hz refresh rate
  	tmr2_act(led_display);				//install the user handler

  	//initialize the chrono
  	chrono_init();

#if defined(SIG1_PIN)
 	//initialize signal generator
	IO_CLR(SIG1_PORT, SIG1_PIN); IO_OUT(SIG1_DDR, SIG1_PIN);
#endif
}

void loop() {
	static unsigned long loop_cnt=0, tick_disp=0;
	static unsigned long tmp;
	static char dp;

  	//put your main code here, to run repeatedly:
    //digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
    //delay(100);               // wait for a second
    //digitalWrite(led, LOW);   // turn the LED off by making the voltage LOW
    //delay(100);               // wait for a second

#if defined(SIG1_PIN)
		//*****************NOTE*********************
		//****for simulation / code testing only****
		//after a certain passage, generate two pulses
		loop_cnt+=1;						//increment loop count
		if (loop_cnt - tick_disp > TK_CNT) {
			tick_disp+=TK_CNT;				//update time
			//generate the first pulse
			IO_SET(SIG1_PORT, SIG1_PIN); NOP4(); IO_CLR(SIG1_PORT, SIG1_PIN);
			//PORTB |= (1<<1); PORTB &=~(1<<1);
			//PORTB ^= (1<<1);
			NOP64();						//time elapsed between the two pulses
			NOP64();						//time elapsed between the two pulses
			NOP64();						//time elapsed between the two pulses
			NOP64();						//time elapsed between the two pulses
			//generate the 2nd pulse
			IO_SET(SIG1_PORT, SIG1_PIN); NOP4(); IO_CLR(SIG1_PORT, SIG1_PIN);
			//PORTB |= (1<<1); PORTB &=~(1<<1);
		}
		//******************end**********************
#endif
		//detecting the 2nd pulse
		//if digit 4's decimal point is on, need to reset the chip
		if (chrono_state == 1) lRAM[3]|=0x80;	//turn on the decimal point for digit 4, indicating that the first pulse has arrived
		//chrono_state = 2; t0cnt=400;			//for debugging
		
		//update display if new data has arrived		
		if (chrono_state == 2) {				//2->new data is available
			chrono_state = 0;					//reset tick_new -> 0=no new data
			//convert ticks elapsed to speed
			spdx1k= T2SPD(t0cnt);				//convert ticks to speed

			//form the display buffer
			tmp = spdx1k;
			     if (tmp >10000000ul-1) tmp = 10000000ul-1;					//max display is "9999."
			     if (tmp > 1000000ul-1) {tmp = (tmp + 500) / 1000; dp=3;}	//1000 - 9999 -> display "xxxx.", rounding up
			else if (tmp >  100000ul-1) {tmp = (tmp +  50) /  100; dp=2;}	//100. - 999. -> display "xxx.x", rounding up
			else if (tmp >   10000ul-1) {tmp = (tmp +   5) /   10; dp=1;}	//10.0 - 99.9 -> display "xx.xx", rounding up
			else if (tmp >    1000ul-1) {tmp = (tmp +   0) /    1; dp=0;}	//1.00 - 9.00 -> display "x.xxx", rounding up
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
