/*
 * LF_PID.c
 *
 * Created: 20-07-2018 17:37:53
 * Author : Vivek Adajania
 */ 
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/delay.h>
#include "usart.h"
double desired_position=3.5 ,kp=150,kd=0,ki=0,integral=0, set_speed=500,saturation_value=500;
 
void lsa_check()
{
	USART_Init(51,0);
	for(int i=0;i<8;i++)
	{
		if(bit_is_set(PIND,i))
		USART_TransmitNumber(i,0);
		_delay_ms(10);
	}
}
void pwm_init()//16 bit timer atmega128
{
	TCCR1A|=(1<<WGM11 | 1<<COM1A1 | 1<<COM1B1);
	TCCR1B|=(1<<WGM13 | 1<<WGM12 | 1<<CS10);
	DDRB|=(1<<PINB5 | 1<<PINB6); //motor pwm
	ICR1=1000;
}
double ir_reading()
{
	double current_position=0 , n=0;
	
	for(int i=0;i<8;i++)							// line sensor weightage from 0 to 7 (left to right)
	{
		if(bit_is_set(PIND,i))						// n is number of IR ON
		{
			current_position=current_position+i;
			n++;
		}
	}
	if(n!=0)
	return current_position/n;
	else 
	return current_position;
}
 double error(double current_position)
{
	double current_error; 
	
	current_error=desired_position-current_position;
	return current_error;
}
double pid(double current_error)
{
	
	double correction , previous_error=0 ,propotional, derivative;
	
	propotional=kp*current_error;
	
	integral=integral+ki*current_error;
	
	derivative=kd*(current_error-previous_error);
	
	previous_error=current_error;
	
	correction=propotional+integral+derivative;
	
	if(correction>saturation_value)
	correction=saturation_value;
	else if(correction<-saturation_value)
	correction=-saturation_value;
	else;
	
	
	return correction;
}
void directions(int x)
{
	switch(x)
	{
		case -1 :
		PORTE|=(1<<PINE7);
		PORTB&=~(1<<PINB0);
		break;						//hard right
		
		case 0 :
		PORTB|=(1<<PINB0);
		PORTE|=(1<<PINE7);			//forward
		break;
		
		case 1 :
		PORTE&=~(1<<PINE7);
		PORTB|=(1<<PINB0);		//hard left
		break;
	}
}
void me_init()
{
	DDRD=0x00;
	DDRB|=(1<<PINB0);
	DDRE|=(1<<PINE7);
	directions(0);
	
}

int main(void)
{
   USART_Init(51,0);
   pwm_init();
   me_init();
	int c=0;   
																			
   while(1)
   {
	   
	   c=(int)pid(error(ir_reading()));		//OCR1B -right //OCR1A- left
										   //direction1-left(E7) direction2-right(B0)
	   OCR1B=set_speed+c;
	   OCR1A=set_speed-c;
	   _delay_ms(10);						//pid frequency							
	  /* if(c==saturation_value)
	   {
		   OCR1A=800;
		   OCR1B=800;							//hard left
		   directions(1);
		   while(!(bit_is_set(PIND,3) && bit_is_set(PIND,4)));
		   goto x;	
	   }
	   else if(c==-saturation_value)
	   {
		   OCR1A=800;
		   OCR1B=800;							//hard right
		   directions(-1);
		   while(!(bit_is_set(PIND,3) && bit_is_set(PIND,4)));
		   goto x;	
	   }
	   else
	   {
		   OCR1B=500+c;
		   OCR1A=500-c;
		   directions(0);
	   }	*/
		   
		   
		 /*  USART_TransmitNumber(ir_reading(),0);	
		   USART_Transmitchar('_',0);
		   USART_TransmitNumber(error(ir_reading()),0);
		   USART_Transmitchar('_',0);
		   USART_TransmitNumber(500+pid(error(ir_reading())),0);
		   USART_Transmitchar('_',0);
		   USART_TransmitNumber(500-pid(error(ir_reading())),0);
		   USART_Transmitchar('_',0);*/
		 						
	}
   }


