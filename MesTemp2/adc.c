/*
 * adc.c
 *
 * Created: 2021/01/29 12:12:46
 *  Author: kuras
 */ 
#include <avr/io.h>


void setup_adc(int ch)
{
	ADCSRA |= 0b111;
	ADMUX = 0b01000000 + ch; // vref = avcc
}

void disable_adc()
{
	ADCSRA &= ~(1<<ADEN);
}

void enable_adc()
{
	ADCSRA |= (1<<ADEN);
}

uint16_t exec_adc()
{
	ADCSRA |= (1<<ADSC);
	while( (ADCSRA & (1<<ADSC)) == 0);
	uint16_t val = ADCL;
	val |= ADCH << 8;
	return val;
}

