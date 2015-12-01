#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>

#define F_CPU 20000000UL
#define UART_BAUD_RATE 115200

#define SERVO 1//The servo is PB1
#define M1 5//The motor forward is PB3 
#define M2 6//The motor backward is PD3
#define M2_top 7// PD7s
#define M1_top 0//PB0

#define ST1 3//PB3 pause led
#define ST2 2//PB2 direction led
#define ST3 2//PD2 finish line led

#define SW1 4//PD4 switch
#define SPEED 3//PD3

#define EDGE_SENS_1 4 //LEFT EDGE SENSOR FOR FINISH LINE DETECTIONSENSOR
#define EDGE_SENS_2 5 //RIGHT EDGE SENSORS FOR FINISH LINE DETECTION
/*
1.0ms=2500 ticks
0.1ms=250 ticks
0.02ms=50 ticks
0.01ms=25 ticks
0.0004ms=1 tick
*/
#define ANGLE_MIN 3200 
#define ANGLE_MAX 4700 //do NOT touch, exactly ?? ms.
#define ANGLE_MID 3950 
#define OFFSET 0
#define ANGLE_RANGE ANGLE_MAX-ANGLE_MID
#define PSAT 255//p saturation
#define NSAT 0//n saturation
#define MIDPOINT ((PSAT+NSAT)>>1)
#define RANGE ((PSAT-NSAT)>>1)
#define LIMITTIME 300 //in # of ticks. correspond to the SLOWEST speed

int variable=8;

/*
The sensor looks like this:
11 1  2 22
OO|O  O|OO
CB A  A BC
angle_min is the minimum number of clicks that the servo can handle (i.e. its 
minimum position)
angle_max is the maximum number of clicks that the servo can handle (i.e. its 
maximum position)
zero is the point where the two sensors are reading the same thing. If it is 
not correctly set, the rover is going to be slightly
off center, by up to a few millimeters. Thus, the value will be set to 127, 
the middle of the ADC range
*/

inline void init(void);
inline void switch_init(void);
inline void timer1(void);
inline void timer2(void);
inline void adc_init(void);
inline void timer0(void);
int16_t spd_timer=0, time=100, time_demand=100, error=0;
volatile int8_t adc[6];
volatile int8_t old_adc[6];
volatile int16_t term0=0, term1=0, term2=0, term3=0, term4=0, term5=0;
volatile uint8_t power=0;
volatile uint16_t spd_i=0;
volatile int16_t power16=0, p_derivative=0, old_time=0, p_error=0, old_error=0;
volatile int32_t p_int;
volatile int servo=ANGLE_MID;		//the servo has to be 16 bit because the timer is 16 bit
volatile int f_sensor=0, old_f_sensor=0, b_sensor, old_b_sensor;		//the front sensor readings
volatile int16_t correction=0;	//the correction angle
volatile int32_t proportional_angle, proportional_offset=0, old_proportional_angle, old_proportional_offset;
volatile int integral=0;
volatile int derivative=0;
volatile uint8_t st_reg=0;	//the register of status LEDs
volatile static unsigned char i1, i0, current;
volatile int i2=0;	//i2 for chaanging direction. 16 bit
volatile int power_count=0;

volatile struct rg {
	unsigned int pause	  	:1;	//paused or not
	unsigned int sw1	 	:1;	//the old value of the SW1
	unsigned int bounce		:1; //debouncing is happening
	unsigned int turn_l0	:1;
	unsigned int turn_r0	:1;
	unsigned int turn_l1	:1;
	unsigned int turn_r1	:1;
	unsigned int turn_l2	:1;
	unsigned int turn_r2	:1;
	unsigned int turn_l3	:1;
	unsigned int turn_r3	:1;
	unsigned int turn_l4	:1;
	unsigned int turn_r4	:1;
	unsigned int turn_l5	:1;
	unsigned int turn_r5	:1;
	unsigned int dir		:1;
	unsigned int old_dir	:1;
	unsigned int finish		:1;
	unsigned int adc_rdy	:1; //adc conversion ready
	unsigned int transmit	:1;	//uart transmission ready
	unsigned int outside	:1;
	unsigned int turns		:6; //max 63
	unsigned int cases		:2;  
	unsigned int perp		:1; //perpendicular to the track flag
	unsigned int stage1		:1;
	unsigned int stage2		:1;
	unsigned int stage3		:1;
	unsigned int stage		:4;
	} reg;

ISR(TIMER0_OVF_vect)//1200Hz motor clock 0.8ms ticks
{
	if (((PINB&(1<<EDGE_SENS_1))==0)&&((PINB&(1<<EDGE_SENS_2))==0)&&(reg.finish==0)&&(reg.pause==0))//if the pins are low
	{
		uart0_puts("finish");
		reg.finish=1;
		i2=0;
	}
	if (reg.finish) {i2++;}
	spd_timer++;
		if(spd_timer>LIMITTIME){time=LIMITTIME; spd_timer=0;}
	if (reg.finish==0){PORTD|=(1<<ST3);}
	else {PORTD&=~(1<<ST3);}
	/*Speed feedback control code*/
	#define DIV 100
	
	p_error=2000/time_demand-2000/time;
	p_derivative=p_error-old_error;
	old_error=p_error;

	#define THR 15
	if((time_demand-time)<-THR)
	{
		power_count++;
	}
	else if ((time_demand-time)>THR)
	{
		power_count-=2;
	}
	#undef THR
	if(power_count<0){power_count=0;}
	else if (power_count>255*DIV) {power_count=255*DIV;}
	power16=3*p_error/4+p_derivative+power_count/DIV;
	if (power16<100&&power16>20)
	{
		power=power16;
	}
	else if (power16<20){power=20;}
		else if (power16>100){power=100;}
	if ((reg.pause==1)||(reg.dir!=reg.old_dir))
	{
		reg.old_dir=reg.dir;
		OCR0A=0;// turn off the other motor driver
		OCR0B=0;// turn off the other motor driver
		PORTB&=~(1<<M1_top);
		PORTD&=~(1<<M2_top);
	}
	else
	{
		if (reg.dir==0)
		{
			/*Turn off the other motor pins*/
			OCR0A=0;// turn off the other motor driver
			PORTD&=~(1<<M2_top);
			/*Turn on the current motor pins*/
			OCR0B=power;  		//turn on the power to the motor
			DDRD|=(1<<M1);		//turn on the motor pin
			DDRB|=(1<<M1_top);	//turn on the top motor pin
			PORTB|=(1<<M1_top);	//turn on the top motor pin
			PORTB&=~(1<<ST2);	//turn off direction LED
		}
		else
		{
			/*Turn off the other motor pins*/
			OCR0B=0;  //turn off the power to the motor
			PORTB&=~(1<<M1_top);
			/*Turn on the current motor pins*/
			OCR0A=power;  		//motor2 (another direction)
			DDRD|=(1<<M2);		//turn on the motor pin
			DDRD|=(1<<M2_top);	//turn on the top motor pin
			PORTD|=(1<<M2_top);	//turn on the top motor pin
			PORTB|=(1<<ST2);	//turn on direction LED
		}
	}
}//end motor clock

ISR(TIMER1_OVF_vect)// 50Hz servo clock
{
	if((servo-ANGLE_MID)<0){servo=ANGLE_MID + ((servo-ANGLE_MID)*5/4);} //have to put more effort to turn to the left
	if(servo<ANGLE_MIN){servo=ANGLE_MIN;}
	if(servo>ANGLE_MAX){servo=ANGLE_MAX;}
	OCR1A=servo;
	i1++;
	if(i1==2){reg.bounce=0;}//after 20-40 ms, turn off the debouncer
}//end 50Hz servo cock
ISR(TIMER2_OVF_vect)//1200Hz shiftout clock
{
	ADCSRA|=1<<ADSC;
}

ISR(PCINT2_vect)//speed, switch and handshake interrupt
{
	if (PIND&(1<<PIND3))  //speed reading
	{
		time=(time*7+spd_timer*1);
		time=time/8;
		spd_timer=0;
	}//end speed reading
	if (reg.finish==1)
		{
			reg.turns++;
		}
	else {reg.turns=0;}
	unsigned char port;//can be merged with the bottom one
	port=PIND&(1<<SW1);
	if(reg.bounce!=1){
		if(reg.sw1){
			if(~(port&1<<SW1)&1<<SW1)
				{
					reg.pause^=1;//toggle the pause with SW1
				}// end if SW1
		}// end if sw1
		reg.sw1=(port&(1<<SW1))>>SW1;
		reg.bounce=1;
		i1=0;
	}// end if (reg.bounce!=1)
}//end speed, switch and handshake interrupt

ISR(ADC_vect)//this is the interrupt triggered by the adc
{
	int16_t temp_adc;
	/*The current value of the ADC is written into the array*/
	current=ADMUX&(1<<MUX0|1<<MUX1|1<<MUX2|1<<MUX3);
	temp_adc=(ADCH-MIDPOINT);//this gives a +/- value
	/*The next two lines scale the ADC to always be in the range
	-128 to 127, regardless of the NSAT and PSAT values*/
	temp_adc=temp_adc*(255-MIDPOINT);
	temp_adc=temp_adc/RANGE;
	if((temp_adc>=(-128))&&(temp_adc<=(127)))
	{
		adc[current]=temp_adc;
	}
	else{
		if(temp_adc>127){adc[current]=127;}
		else{adc[current]=-128;}
	}
	if(current<5){++current;}//increment
	else{current=0;}//reset
	ADMUX&=~(1<<MUX0|1<<MUX1|1<<MUX2|1<<MUX3);//zero the MUX register
	ADMUX|=current;
	reg.adc_rdy=1;
}

int main(void)
{	
	reg.pause=1;//start paused
	reg.sw1=1;//we assume that the switches are depressed on startup
	reg.bounce=1;
	reg.finish=1;
	reg.stage=10;
	reg.turns=0;
	reg.dir=0;
	char buf[10];
	uint8_t buf_count;
	init();
	sei();
	uart0_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	while(1)
	{
		if(reg.adc_rdy)//ADC conversion done
		{
			if(current==3)//all the front sensors have been polled
			{
				/*Here begins the code processing the front sensor*/
				/*Going to positive side of the front sensor*/
				//flag conditions
				
				#define OFST 50 //this is the range from zero that is assumed to be essentially zero
				if (adc[0] == 127)												{reg.turn_l0=1; reg.turn_r0=0;}
				if ((adc[0]<127)&&(adc[1]>-OFST)&&(adc[1]<OFST)&&(adc[2]<OFST)&&(adc[2]>-OFST)&&(reg.turn_l1==0)&&(reg.turn_l2==0)){reg.turn_l0=0;}
				if (adc[1]==127)												{reg.turn_l1=1;reg.turn_r1=0;}
				if ((adc[1]>OFST)&&(adc[1]<127)&&(adc[0]==127))					{reg.turn_l1=0;}
				if (adc[2]==127)												{reg.turn_l2=1;reg.turn_r2=0;}
				if ((adc[2]>OFST)&&(adc[2]<127)&&(adc[0]==127)&&(adc[1]==127))	{reg.turn_l2=0;}
				/*Going to negative side of the front sensor*/
				//flag conditions
				if (adc[0] == -128)												{reg.turn_r0=1;reg.turn_l0=0;}
				if ((adc[0]>-128)&&(adc[1]<OFST)&&(adc[1]>-OFST)&&(adc[2]<OFST)&&(adc[2]>-OFST)&&(reg.turn_r1==0)&&(reg.turn_r2==0)){reg.turn_r0=0;}
				if (adc[1]==-128)												{reg.turn_r1=1;reg.turn_l1=0;}
				if ((adc[1]<-OFST)&&(adc[1]>-128)&&(adc[0]==-128))				{reg.turn_r1=0;}
				if (adc[2]==-128)												{reg.turn_r2=1;reg.turn_l2=0;}
				if ((adc[2]<-OFST)&&(adc[2]>-128)&&(adc[0]==-128)&&(adc[1]==-128)){reg.turn_r2=0;}
				//determine governing equationg dependent on the flags for the front sensor
				if (reg.turn_l0==1)			{term0=255-adc[0];}
				else 	if (reg.turn_r0==1)	{term0=-255-adc[0];}
						else				{term0=adc[0];}
				if (reg.turn_l1==1)			{term1=255-adc[1];}
				else 	if (reg.turn_r1==1)	{term1=-255-adc[1];}
						else				{term1=adc[1];}
				if (reg.turn_l2==1)			{term2=255-adc[2];}
				else 	if (reg.turn_r2==1)	{term2=-255-adc[2];}
						else				{term2=adc[2];}
				f_sensor = term0 + term1 + term2;
				derivative=f_sensor-old_f_sensor;
			}//end front sensor processing
			if(current==0)//the sensors have restarted counting
			{
				/*Here begins the code processing the back sensor*/
				//flag conditions
				if (adc[3] == 127)											{reg.turn_l3=1; reg.turn_r3=0;}
				if ((adc[3]<127)&&(adc[4]>-OFST)&&(adc[4]<OFST)&&(adc[5]<OFST)&&(adc[5]>-OFST)&&(reg.turn_l4==0)){reg.turn_l3=0;}
				if (adc[4]==127)											{reg.turn_l4=1;reg.turn_r4=0;}
				if ((adc[4]>OFST)&&(adc[4]<127)&&(adc[3]==127))				{reg.turn_l4=0;}
				if (adc[5]==127)											{reg.turn_l5=1;reg.turn_r5=0;}
				if ((adc[5]>OFST)&&(adc[5]<127)&&(adc[3]==127)&&(adc[4]==127)){reg.turn_l5=0;}

				//flag conditions
				if (adc[3] == -128)											{reg.turn_r3=1;reg.turn_l3=0;}
				if ((adc[3]>-128)&&(adc[4]<OFST)&&(adc[4]>-OFST)&&(adc[5]<OFST)&&(adc[5]>-OFST)&&(reg.turn_r4==0)){reg.turn_r3=0;}
				if (adc[4]==-128)											{reg.turn_r4=1;reg.turn_l4=0;}
				if ((adc[4]<-OFST)&&(adc[4]>-128)&&(adc[3]==-128))			{reg.turn_r4=0;}
				if (adc[5]==-128)											{reg.turn_r5=1;reg.turn_l5=0;}
				if ((adc[5]<-OFST)&&(adc[5]>-128)&&(adc[3]==-128)&&(adc[4]==-128)){reg.turn_r5=0;}
				//determine governing equationg dependent on the flags back sensor
				if (reg.turn_l3==1)			{term3=255-adc[3];}
				else 	if (reg.turn_r3==1)	{term3=-255-adc[3];}
						else				{term3=adc[3];}
				if (reg.turn_l4==1)			{term4=255-adc[4];}
				else 	if (reg.turn_r4==1)	{term4=-255-adc[4];}
						else				{term4=adc[4];}
				if (reg.turn_l5==1)			{term5=255-adc[5];}
				else 	if (reg.turn_r5==1)	{term5=-255-adc[5];}
						else				{term5=adc[5];}
				#undef OFST //just so that it doesnt clutter up the code
		
				b_sensor = term3 + term4 + term5;
			}//end back sensor processing
			
			proportional_offset=(f_sensor+b_sensor)/6;
			integral=integral+proportional_offset/32;
			old_proportional_angle=proportional_angle;
			old_proportional_offset=proportional_offset;
			old_f_sensor=f_sensor;

			if((reg.turn_l2==0)&&(reg.turn_l5==0)&&(reg.turn_r2==0)&&(reg.turn_r5==0))
			{
				proportional_angle=b_sensor-f_sensor;
				reg.outside=0;
			}
			else if((f_sensor<730&&f_sensor>-730)&&(b_sensor<730&&b_sensor>-730))
			{
				reg.outside=1;
			}
			
			if(reg.outside)
			{
				if((f_sensor<730&&f_sensor>-730)||(b_sensor<730&&b_sensor>-730))
				{
					proportional_angle=b_sensor-(3*f_sensor/2);
				}
			}
			/*keep the proportional term in bounds*/ 
			if(proportional_angle>ANGLE_RANGE){proportional_angle=ANGLE_RANGE;}
			if(proportional_angle<(-ANGLE_RANGE)){proportional_angle=-ANGLE_RANGE;}
			
			/*keep the integral term in bounds*/
			if(integral>ANGLE_RANGE){integral=ANGLE_RANGE;}
			if(integral<(-ANGLE_RANGE)){integral=-ANGLE_RANGE;}
			//keep the derivative term in bounds*/
			if(derivative>ANGLE_RANGE){derivative=ANGLE_RANGE;}
			if(derivative<(-ANGLE_RANGE)){derivative=-ANGLE_RANGE;}
			if ((f_sensor>500||f_sensor<-500)&&(proportional_angle<200)&&(proportional_angle>-200)) //sharp turn condition
			{
				correction=f_sensor/5-(4*proportional_angle)/12 + integral/14 + 16*derivative;	//for sharp turn increase gain on f_sensor, reduce on angle
			}
			else
			{
				correction=f_sensor/12-(6*proportional_angle)/12 + integral/14 + 16*derivative;	//otherwise reduce gain on f_sensor, increase on angle
			}
			reg.adc_rdy=0;//this ensures that we only calculate these results if there are new values in the adc
		}//end adc_rdy
		/*turn around code */
		#define FT2 32	//how much to travel before starting 1st stage of turning around
		if (reg.finish)
		{			
			if (((reg.stage!=10)&&(PINB&(1<<EDGE_SENS_1))&&(PINB&(1<<EDGE_SENS_2))&&(f_sensor<300)&&(f_sensor>-300)&&(reg.turns>15))||(reg.turns>=FT2))
			{
				if(reg.turns>15&&reg.turns<FT2)
				{	//determine case, to which side to turn around
					reg.turns=FT2; //make sure it doesn't enter the same condition again
					if (proportional_angle<0)
					{
						reg.cases=0;		//angle to the left
						servo=ANGLE_MIN;
					}
					else if (proportional_angle>0)
					{
						reg.cases=1;	//angle to right, rover on right of the track
						servo=ANGLE_MAX;
					}
				}
				/*turn around*/
				else if (reg.turns>=FT2) //turning around procedure
				{
					if (reg.cases==0)	//points left
					{
						if ((PINB&(1<<EDGE_SENS_2))&&(reg.stage==0))		//if right edge sensor is on black
						{													//keep going
							reg.dir=0;
							reg.pause=0;
						}
						else if (((PINB&(1<<EDGE_SENS_2))==0)&&(reg.stage<1))	//once right edge sensor is on white
						{
							reg.pause=1;			//pause
							i2=0;					//reset counter
							servo=ANGLE_MAX;		//steer to right
							reg.dir=1;				//go back
							reg.stage=1;
						}
						else if ((reg.stage==1)&&(i2>600))	//after counter passes 600
						{
							reg.stage=2;
							reg.pause=0;			//start motor. so now it will start going back
						}
						else if (((PINB&(1<<EDGE_SENS_1))==0)&&(reg.stage==2))	//once left edge sensor is on white	
						{
							reg.pause=1;		//pause
							i2=0;				//reset counter
							reg.dir=0;			//go forward
							reg.stage=3;
							servo=ANGLE_MIN;	//steer to left
						}
						else if ((reg.stage==3)&&(i2>600))	//after counter passes 600
						{
							reg.pause=0;		//start going forward
							reg.turns=FT2;		//reset turn counter. but start from FT2 not to enter unwanted cycles
							reg.stage=4;
						}
						else if ((reg.stage==4)&&(reg.turns>(FT2+3)))	//after 3 turns
						{
							reg.pause=1;		//pause
							i2=0;				//reset counter
							reg.dir=1;			//go backwards
							servo=ANGLE_MAX;	//steer to right
							reg.stage=5;
						}
						else if (((PINB&(1<<EDGE_SENS_1))==0)&&(reg.stage==5))	//once left edge sensor is on white
						{						//this cycle ir triggered next after else if ((reg.stage==5)&&(i2>600)) cycle below
							reg.pause=1;		//pause
							i2=0;				//reset counter
							reg.dir=0;			//go forward
							reg.stage=6;
							servo=ANGLE_MIN;
						}
						else if ((reg.stage==5)&&(i2>600))	//after counter passes 600
						{
							reg.pause=0;		//start going backwards steering to the right
						}						//this cycle is triggered next after the else if ((reg.stage==4)&&(reg.turns>(FT2+3))) cycle above
						else if ((reg.stage==6)&&(i2>600))	//after counter passes 600
						{
							reg.pause=0;		//start going forward
							reg.turns=FT2;		//reset turns to the starting constant
							reg.stage=7;
						}
						else if ((reg.stage==7)&&(reg.turns>(FT2+5)))	//after 5 turns
						{
							reg.stage=10;		//condition for start following the line while still in turning around mode
							reg.turns=0;		//reset turns to enter the cycle for line following
						}
					} //end of case 0
					else if (reg.cases==1)		//point to the right
					{
						if ((PINB&(1<<EDGE_SENS_1))&&(reg.stage==0))	//if left edge sens is on black
						{												//keep going
							reg.dir=0;
							reg.pause=0;
						}
						else if (((PINB&(1<<EDGE_SENS_1))==0)&&(reg.stage<1))	//once left edge sensor is on white
						{
							reg.pause=1;			//pause
							i2=0;					//reset counter
							servo=ANGLE_MIN;		//steer to left
							reg.dir=1;				//go back
							reg.stage=1;
						}
						else if (((PINB&(1<<EDGE_SENS_2))==0)&&(reg.stage==1))	//once right sensor is on white
						{	//this cycle is entered after else if ((reg.stage==1)&&(i2>600)) cycle below
							reg.pause=1;			//pause
							i2=0;					//reset counter
							reg.dir=0;				//go forward
							servo=ANGLE_MAX;		//steer to right
							reg.stage=2;
						}
						else if ((reg.stage==1)&&(i2>600))	//after counter passes 600
						{	//this cycle is entered next after else if (((PINB&(1<<EDGE_SENS_1))==0)&&(reg.stage<1)) cycle above
							reg.pause=0;		//start going backwards
						}
						else if ((reg.stage==2)&&(i2>600))	//after counter passes 600
						{	//this cycle is entered after else if (((PINB&(1<<EDGE_SENS_2))==0)&&(reg.stage==1)) cycle above
							reg.pause=0;			//start going forwards
							reg.turns=FT2;			//reset turns
							reg.stage=3;
						}
						else if ((reg.stage==3)&&(reg.turns>(FT2+3)))	//after 3 turns
						{
							reg.pause=1;			//pause
							i2=0;					//reset counter
							reg.dir=1;				//go backwards
							servo=ANGLE_MIN;		//steer to left
							reg.stage=4;
						}
						else if (((PINB&(1<<EDGE_SENS_1))==0)&&(reg.stage==4))	//once the left edge sensor is above white
						{	//this cycle is entered after else if ((reg.stage==4)&&(i2>600)) cycle below
							reg.pause=1;			//pause
							i2=0;					//reset counter
							reg.dir=0;				//go forwards
							reg.stage=5;	
							servo=ANGLE_MAX;		//steer to right
						}
						else if ((reg.stage==4)&&(i2>600))	//after counter passes 600
						{	//this cycle is entered after else if ((reg.stage==3)&&(reg.turns>(FT2+3))) cycle above
							reg.pause=0;			//start going backwards
						}
						else if ((reg.stage==5)&&(i2>600))	//after counter passes 600
						{	//this cycle is entered after else if (((PINB&(1<<EDGE_SENS_1))==0)&&(reg.stage==4)) cycle above
							reg.pause=0;			//start going backwards
							reg.turns=FT2;			//reset turns
							reg.stage=6;
						}
						else if ((reg.stage==6)&&(reg.turns>(FT2+7)))	//after 7 turns
						{
							reg.stage=10;		//condition for start following the line while still in turning around mode
							reg.turns=0;		//reset turns to enter the cycle for line following
						}
					} //end of case 1
				}// end turning around procedure
			}//end code for the rover once it is alligned
			else if (reg.pause==0)// alignment procedure after finish line detection
			{
				servo=correction+ANGLE_MID+OFFSET; //continues correction even after crossing the finish
				time_demand=150;		//sets slower speed
				if ((reg.turns>20)&&(reg.stage==10))	//special condition for turning off the finish flag
				{
					reg.finish=0;
					reg.stage=0;
				}
			}//end alignment procedure while after passed the finish line
		}// end finish line flag handling
		else
		{
			time_demand=120;	//speed for normal track following
		}
		if(reg.pause)//pause
		{        
			/*
			 * Get received character from ringbuffer
			 * uart_getc() returns in the lower byte the received character and 
			 * in the higher byte (bitmask) the last receive error
			 * UART_NO_DATA is returned when no data is available.
			 */
			power_count=25*DIV;	//set initial power count value to accelerate from rest faster
			power=0;		//stop the rover
			uint16_t c;
			c = uart0_getc();
			if ( c & UART_NO_DATA )
			{
				/*no data available from UART*/
			}
			else
			{
				/*new data available from UART*/
				if(reg.transmit)
				{
					if((c&0xff)==13) //transmit is complete upon carriage return
					{
						reg.transmit=0;
						if(buf_count)
						{
							buf[buf_count]='\0';
							variable=atoi(buf);
						}						
						uart0_puts(itoa(variable, buf, 10));
						buf_count=0;
					}//end if
					else
					{
						if(buf_count<9)//smaller than 9 because there must be space for the \0 character
						{
						buf[buf_count]=(char)c&0xff;
						buf_count++;
						}
						else
						{
							uart0_puts("Longass number");
							reg.transmit=0;
							buf_count=0;
						}
					}
				}	//end if (reg.transmit)
				if((c&0xff)=='N')
				{
					reg.transmit=1;
				}
			}
				PORTB|=1<<ST1;//turn on the pause LED
			if (reg.finish==0)
			{
				servo=ANGLE_MID+OFFSET;	//only want to set servo to middle if not turning around after finish line
			}
		}// end if (reg.pause)
		else//not paused
		{
			if (reg.finish==0)
			{
				servo=correction+ANGLE_MID+OFFSET;	//if not turning around, then follow the line
			}
			PORTB&=~(1<<ST1);//turn off the pause LED
		}//end unpaused case
	}//end while(1)
}

inline void init(void)
{
	ICR1=50000;//should give exactly 50Hz with a prescaler of 8
	timer0();
	timer1();
	timer2();
	adc_init();
	ADCSRA|=1<<ADSC;
	/* Setting the Output Pins*/
	DDRB|=1<<SERVO|1<<M1_top|1<<ST1|1<<ST2;
	DDRD|=1<<M2_top;
	PORTD&=~(1<<M1|1<<M2_top|1<<M2);
	PORTB&=~(1<<SERVO|1<<M1_top);
	DDRD|=1<<ST3;
	/*Setting the input pins*/
	DDRB&=~(1<<EDGE_SENS_1|1<<EDGE_SENS_2);
	PORTB&=~(1<<EDGE_SENS_1|1<<EDGE_SENS_2);
	DDRD&=~(1<<M2);
	DDRD&=~(1<<M1);
	switch_init();
}
inline void switch_init(void)
{
	PCICR|=1<<PCIE2;//enable the level change interrupt for the switch
	PCMSK2|=(1<<SW1|1<<SPEED); //allow interrupts for switches and speed
	DDRD&=~(1<<SW1); //set pins to input
	PORTD|=(1<<SW1);
}
inline void timer0(void) //1200 Hz for ADC & shift out
{
	TCCR0A&=0b00001111;	//Clear the COM0A/B bits
	TCCR0A|=0b10100000;	//Clear OC0B on Match

	TCCR0B&=0b11111000;
	TCCR0B|=0b00000011; //Set prescaler to 64 for 1kHz ADC frequency

	TCCR0A&=0b11111100; //Clear the WGM bits
	TCCR0B&=0b11110111;	//Clear the WGM bits
	
	TCCR0A|=0b00000011;//Fast PWM
	TIMSK0|=1<<TOIE0;
}
inline void timer1(void) //50 Hz servo
{
	OCR1A=ANGLE_MID;
	OCR1B=0;
	
	TCCR1B&=~0b00000111;//clear the prescaler bits
	TCCR1B|=0b00000010;//set prescaler to 8
	
	TCCR1A&=0b00001111;//clear the PinA/B mode bits
	TCCR1A|=0b10000000;//set the PinA mode to "Clear on match"
	
	TCCR1A&=0b11111100;//clear the WGM bits
	TCCR1B&=0b11100111;//clear the WGM bits
	
	TCCR1A|=0b00000010;//set WGM to 14
	TCCR1B|=0b00011000;//set WGM to 14
	
	TIMSK1&=0b00000000;//clear the interrupt mask bits
	TIMSK1|=1<<TOIE1;//enable interrupt for this timer
}
inline void timer2(void)  // ~10 kHz for motor pwm
{
	OCR2A=0;//just so that our motor doesnt jumpstart
	OCR2B=127;
	
	TCCR2A&=0b00001111;	//Clear the COM2A/B bits
	TCCR2A|=0b00000000;	//Clear OC2A on Match
	
	TCCR2B&=0b11111000;//clear the prescaler bits
	TCCR2B|=0b00000100;//set the prescaler to 64
	
	TCCR2A&=0b11111100;//clear the WGM bits
	TCCR2B&=0b11110111;//clear the WGM bits
	
	TCCR2A|=0b00000011;//set WGM to Fast
	
	TIMSK2|=1<<TOIE2;
}

inline void adc_init(void)
{
	ADCSRA|=1<<ADEN;//enables the ADC

	ADMUX&=0b11110000;//clear the bottom bits (MUX3:0)
	DIDR0|=1<<ADC0D;//ADC 0
	
	ADMUX&=0b00111111;//clear the top bits
	ADMUX|=1<<REFS0;//AVcc reference
	
	ADMUX|=1<<ADLAR;//left adjust
	
	ADCSRA|=1<<ADIE;//interrupt enable
	ADCSRB&=0b11111000;//clear the bottom bits (ADTS2:0)
	
	ADCSRA&=0b11111000;//clear the bottom bits (ADPS2:0)
	ADCSRA|=1<<ADPS2|1<<ADPS1|1<<ADPS0;//prescaler 128
}

void write_led(unsigned char* led_reg_ptr, int led_number, unsigned char status)
{
	if(led_number<=7)
	{			
		if(status){(*led_reg_ptr)|=1<<led_number;}
		else{(*led_reg_ptr)&=~(1<<led_number);}
	}
}