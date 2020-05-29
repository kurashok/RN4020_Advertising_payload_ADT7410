/*
 * iic.c
 *
 * Created: 2020/05/14 15:42:19
 *  Author: kuras
 */
#define F_CPU 1000000

#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <stdio.h>
//#include <avr/sleep.h>
#include <util/delay.h>
#include "iic.h"

void setup_iic(void)
{
	I2C_DDDR |=(1<<I2C_SCL)|(1<<I2C_SDA);     // set DDR as output port
	I2C_DPORT &=~((1<<I2C_SCL)|(1<<I2C_SDA)); // PORT set low

//	_delay_ms(40);

//	iic_start();
//	iic_send(0xFF);
//	iic_start();
//	iic_stop();
}

static void iic_delay (void)
{
	_delay_us(5);
}

/* Generate start condition on the IIC bus */
void iic_start (void)
{
	SDA_HIGH;
	iic_delay();
	SCL_HIGH;
	iic_delay();
	SDA_LOW;
	iic_delay();
	SCL_LOW;
	iic_delay();
}

/* Generate stop condition on the IIC bus */
void iic_stop (void)
{
	SDA_LOW;
	iic_delay();
	SCL_HIGH;
	iic_delay();
	SDA_HIGH;
	iic_delay();
}

/* Send a byte to the IIC bus */
unsigned char iic_send (uint8_t dat)
{
	uint8_t b = 0x80;
	unsigned char ack;

	do {
		if( dat & b )
		{	/* SDA = Z/L */
			SDA_HIGH;
		}
		else
		{
			SDA_LOW;
		}
		iic_delay();
		SCL_HIGH;
		iic_delay();
		SCL_LOW;
		iic_delay();
	}while (b >>= 1);
	SDA_HIGH;
	iic_delay();
	SCL_HIGH;
	//	while( SCL_VAL == 0 );
	ack = SDA_VAL;	/* Sample ACK  inverted 7/9/09 */
	iic_delay();
	SCL_LOW;
	iic_delay();
	return ack;
}

unsigned char iic_recv (uint8_t nack)
{
	uint8_t data = 0;
	for( uint8_t i=0; i<8; i++ )
	{
		data <<= 1;
		SCL_HIGH;
		//_delay_us(10);
		if( SDA_VAL != 0 )
		{
			data |= 1;
		}
		iic_delay();
		SCL_LOW;
	}
	iic_delay();
	if( nack )
	{
		SDA_HIGH;
	}
	else
	{
		SDA_LOW;
	}
	iic_delay();
	SCL_HIGH;
	iic_delay();
	SCL_LOW;
	SDA_HIGH;
	iic_delay();
	return data;
}
