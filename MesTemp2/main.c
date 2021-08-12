/*
 *
 * Created: 2020/05/20 11:07:32
 */ 
#define F_CPU 1000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "iic.h"
#include "adc.h"
#include "eeprom.h"

uint16_t  uiTimer_0 ;
// 温度測定間隔(200ms単位)
#define MES_INTERVAL 28
// 温度変換時間(240ms以上)
#define CONVERSION_WAIT 2

// 測定インターバル中か、測定器の変換終了待ちかを示すフラグ
uint8_t bMes_cycle;

#define WDT_8s 9
#define WDT_4s 8
#define WDT_2s 7
#define WDT_1s 6
#define WDT_500ms 5
#define WDT_250ms 4
#define WDT_125ms 3

void setup_WDT( uint8_t delay )
{
	// ウォッチドッグタイマのタイムアウト設定
	if( delay > 9 ) delay = 9;
	uint8_t reg_data = delay & 7;
	if( delay & 0x8 )
	{
		reg_data |= 0x20;
	}

	MCUSR &= ~(1 << WDRF);
	//WDTCSR |= (1 << WDCE) | (1 << WDE); // 設定変更許可
	//WDTCSR = reg_data | 1<<WDE;	// タイムアウト設定
	WDTCSR |= (1 << WDCE) | (1 << WDE); // 設定変更許可
	WDTCSR = reg_data | 0<<WDE;	// タイムアウト設定
	WDTCSR |= (1 << WDIE); // 割り込み許可
}

//#define BAUD_PRESCALE 51 // U2X=0 8MHz 9600B
//#define BAUD_PRESCALE 12 // U2X=0 2MHz 9600B
#define BAUD_PRESCALE 12 // U2X=1 1MHz 9600B

volatile uint8_t bRecieve;
char *waiting_message;

void setup_USART(void){
	// Set baud rate
	UBRR0L = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
	UBRR0H = (BAUD_PRESCALE >> 8);

	// 倍速ビットON
	UCSR0A = 2;
	// Enable receiver and transmitter and receive complete interrupt
	UCSR0B = ((1<<TXEN0)|(1<<RXEN0) | (1<<RXCIE0));
	UCSR0C = 0b00000110;
}

void USART_SendByte(uint8_t u8Data){

	// Wait until last byte has been transmitted
	while((UCSR0A &(1<<UDRE0)) == 0);

	// Transmit data
	UDR0 = u8Data;
	
	// clear tx complete flag
	UCSR0A |= (1<<TXC0);
}

void USART_SendStr(const char * str)
{
	uint8_t i = 0;
	while( str[i] != '\0' )
	{
		USART_SendByte(str[i++]);
	}
	
	// wait for tx complete
	while( (UCSR0A & (1<<TXC0)) == 0 );
}

#define RCV_SIZE 100
char RCV_BUF[RCV_SIZE];
char MSG_BUF[RCV_SIZE];
uint8_t RCV_PTR = 0;
volatile uint8_t bline_cmp = 0;

void check_command()
{
	if( strstr(RCV_BUF, waiting_message) != NULL )
	{
		bRecieve = 1;
	}
}

ISR (USART_RX_vect)
{
	char value = UDR0;             //read UART register into value
	if( value == '\x0d' )
	{
	}
	else if( value == '\x0a' )
	{
		RCV_BUF[RCV_PTR] = '\0';
		strcpy((char*)MSG_BUF,(const char*)RCV_BUF);
		bline_cmp = RCV_PTR;
		RCV_PTR = 0;
	}
	else if( isprint(value) )
	{
		RCV_BUF[RCV_PTR] = value;
		if( RCV_PTR < RCV_SIZE-1 )
		{
			RCV_PTR++;
		}
	}
}

int write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	iic_start();
	int ack = iic_send(dev_addr<<1);
	if( ack != 0 )
	{
		return 10;
	}

	ack = iic_send(reg_addr);
	if( ack != 0 )
	{
		return 11;
	}

	ack = iic_send(data);
	if( ack != 0 )
	{
		return 12;
	}

	iic_stop();
	return 0;
}

int set_one_shot(uint8_t dev_addr)
{
	// one-shot mode
	int ack = write_reg(dev_addr, 0x03, 0x20);
	if( ack != 0 )
	{
		return ack; // ack error
	}
	// wait for temperature measure and conversion (typical:240ms)
	//_delay_ms(300);
	return 0;
}

int read_temp( uint8_t dev_addr, double *value )
{
	int ack;

	// read temperature value (r0,r1)
	iic_start();
	ack = iic_send(dev_addr<<1);
	if( ack != 0 )
	{
		return 1; // ack error
	}

	ack = iic_send(0x0);
	if( ack != 0 )
	{
		return 2; // ack error
	}
	
	iic_start();

	ack = iic_send(dev_addr<<1 | 1);
	if( ack != 0 )
	{
		return 3; // ack error
	}

	unsigned char id0 = iic_recv(0);
	unsigned char id1 = iic_recv(1);
	iic_stop();
	int_fast16_t tmp = id0<<8 | id1; // tmpデータは符号付き
	tmp >>= 3;
	*value = (double)tmp/16.0;
	return 0;
}

void temp_to_string( double temp, char *buf )
{
	double t = temp + 0.05; // 小数第２位で四捨五入
	sprintf( buf, "%d.%01d", (int)t, abs((int)((t-(int)t)*10)) );
}

void disable_rxint()
{
	UCSR0B &= ~(1<<RXCIE0);
}

void enable_rxint()
{
	UCSR0B |= (1<<RXCIE0);
}

void send_RN4020_command( const char *cmd, const char *ack )
{
	if( cmd != NULL )
	{
		USART_SendStr(cmd);
	}
	if( ack != NULL )
	{
		waiting_message = (char*)ack;
		bRecieve = 0;
		while( bRecieve == 0 );
	}
}

void check_connection(const char *buf)
{
	static int prv_con = 0;
	if( PINB & 0x4 )
	{
		PORTD |= 0x10; // connect LED on
		PORTB |= 1; // operation mode
		PORTB |= 2; // MLDP mode
		_delay_ms(5);
		USART_SendStr( buf );
		//PORTB &= ~2; // CMD mode
		PORTB &= ~1; // sleep mode
		prv_con = 1;
	}
	else
	{
		PORTD &= ~0x10; // connect LED off
		PORTB |= 1; // operation mode

		if( prv_con == 1 )
		{
			PORTB &= ~2; // CMD mode
		}
		_delay_ms(5);
		send_RN4020_command("A,0064,03e8\x0d\x0a", NULL);
		//_delay_ms(100);
		PORTB &= ~1; // sleep mode

		prv_con = 0;
	}
}

void make_info_mes(int err, double tmp, double vol, char *buf)
{
	int pol = 0;
	if( tmp < 0 ) 
	{
		pol = 1;
		tmp *= -1;
	}
	int t = (tmp + 0.05)*10;
	uint16_t t_hd = (t%10) << 4;
	t /= 10;
	t_hd |= (t%10) << 8;
	t /= 10;
	t_hd |= t << 12;

	int v = (vol + 0.005)*100;
	uint16_t v_hd = (v%10);
	v /= 10;
	v_hd |= (v%10) << 4;
	v /= 10;
	v_hd |= (v%10) << 8;
	v /= 10;
	v_hd |= v << 12;

	sprintf( buf, "N,9999%04x%02x%04x%04x\x0d\x0a", err, pol, t_hd, v_hd );
}

void send_advertise(char *buf)
{
	PORTB |= 1; // operation mode
	PORTB &= ~2; // CMD mode

	_delay_ms(5);
	send_RN4020_command(buf, NULL);
	send_RN4020_command("A,0063,00c8\x0d\x0a", NULL);
//	send_RN4020_command("A,0064,03e8\x0d\x0a", NULL);

	PORTB &= ~1; // sleep mode
}

char mes_buf[100];

void meas()
{
	uiTimer_0--;
	if( uiTimer_0 == 0 )
	{
		if( bMes_cycle == 0 )
		{
			set_one_shot(0x48);
			//if( err != 0 )
			//{
			//	sprintf( buf, "ERROR=%d\r\n", err );
			//	USART_SendStr( buf );
			//	uiTimer_0 = MES_INTERVAL;
			//}
			//else
			{
				bMes_cycle = 1;
				uiTimer_0 = CONVERSION_WAIT;
			}
		}
		else
		{
			uiTimer_0 = MES_INTERVAL;
			bMes_cycle = 0;

			double tmp;
			double  vol = 0.0;
			int err = read_temp(0x48, &tmp);
			//if( err == 0 )
			{
				enable_adc();
				setup_adc(0);
				uint16_t val = exec_adc();
				vol = (double)val * (3.3/1024);
			}
			make_info_mes(err, tmp, vol, mes_buf);
		}
	}
	send_advertise(mes_buf);
}

void exec_cmd(char *msg, char *buf)
{
	for(uint8_t i=0; msg[i]!='\0'; i++)
	{
		if( !isalpha(msg[i])) continue;
		msg[i] = toupper(msg[i]);
	}

	if( strncmp(msg, "POW", 3) == 0 )
	{
		// TX POWER設定
		uint8_t pow = atoi(msg+4);
		write_eeprom( 7, (unsigned char)pow );
		sprintf(buf, "ACK %d\x0d\x0a", pow);
	}
	else if(strncmp(msg,"RR", 2) == 0 )
	{
		// ROM読み込み
		uint16_t adr = strtoul(msg+3, NULL, 16);
		uint16_t dat = read_eeprom(adr);
		sprintf(buf, "ACK %x\x0d\x0a", dat);
	}
	else if( strncmp(msg, "RW", 2) == 0 )
	{
		// ROM書き込み
		uint16_t adr = strtoul(msg+3, NULL, 16);
		uint16_t dat = strtoul(msg+6, NULL, 16);
		write_eeprom( adr, (unsigned char)dat );
		strcpy(buf, "ACK\x0d\x0a");
	}
	else
	{
		sprintf(buf, "ERR %s\x0d\x0a", msg);
	}
}

void setting_mode()
{
	// アドバタイズ開始
	USART_SendStr("a\x0d\x0a");
	// 接続チェック
	bool bConnect = false;
	for(int i=0; i<30; i++)
	{
		PORTD |= (1 << 4);
		_delay_ms(100);
		PORTD &= ~(1 << 4);

		_delay_ms(900);
		if( PINB & 0x4 )
		{
			bConnect = true;
			break;
		}
	}
	// 接続なし
	if( !bConnect )
	{
		// アドバイス停止
		USART_SendStr("y\x0d\x0a");
		return;
	}
	// 接続あり
	PORTD |= (1 << 4);
	_delay_ms(1000);
	PORTD &= ~(1 << 4);
	// MLPDモード設定
	PORTB |= 2;
	// コマンド受付・実行
	sei();
	bline_cmp = 0;
	for(;;)
	{
		// コマンド受け取待ち
		for(;;)
		{
			// 接続が切れたら終了
			if( (PINB & 0x4) == 0 )
			{
				PORTB &= ~2; // cmd mode
				PORTD &= ~(1 << 4); // led off
				return;
			}
			// コマンド受け取ったらループ抜ける
			if( bline_cmp != 0 )
			{
				bline_cmp = 0;
				break;
			}
		}
		// コマンド実行
		char buf[100];
		exec_cmd((char*)MSG_BUF, buf);
		// 戻りを送信
		USART_SendStr(buf);
	}
}

void setup_RN4020()
{
	// B0 : output : WAKE_SW
	DDRB |= 1;
	PORTB |= 1; // WAKE_SW = 1 operation mode

	// B1 : output : CMD_MLDP
	DDRB |= 2;
	PORTB &= ~2; // CMD_MLDP = 0 cmd mode

	// B2 : input : connect indicator
	DDRB &= ~4;

	// C0 : output : status led
	DDRD |= 0x10;
	PORTD |= 0x10; // led = 1

	PORTD |= 1; // RXD端子プルアップ

	_delay_ms(5000);
	send_RN4020_command("SS,00000007\x0d\x0a", NULL);
	send_RN4020_command("SR,12000000\x0d\x0a", NULL);
	send_RN4020_command("R,1\x0d\x0a", NULL);
	_delay_ms(5000);

	PORTD &= ~0x10; // led = 0
}

ISR (WDT_vect)
{
}

void set_tx_power()
{
	uint8_t pow = read_eeprom(7);
	if( pow == 0xff )
	{
		pow = 4;
	}
	if( pow > 8 ) pow = 7;
	char buf[20];
	sprintf(buf,"SP,%d\x0d\x0a", pow);
	send_RN4020_command(buf, NULL);
}

int main(void)
{
	uint8_t cal = read_eeprom(0);
	if(cal != 0xff)
	{
		OSCCAL = cal;
	}
	CLKPR = 0x80;// クロック分周比変更許可ビットON
	CLKPR = 0x03;// 分周比1/8 : クロックは1MHz

	setup_USART();
	DIDR0 = 0xf; // PC0,1,2,3はデジタル入力禁止。ADC入力専用
	PRR |= (1<<PRTWI) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRTIM2);

	sei();			// 割込みを許可する。

	setup_RN4020();
	setting_mode();

	disable_rxint();
	
	set_tx_power();

	setup_iic();
	write_reg(0x48, 0x3, 0x60); // set shutdown mode (13bit mode)

	bMes_cycle = 0;

	uiTimer_0 = 1;
	sprintf( mes_buf, "N,999900000000000000\x0d\x0a" );
	while (1)
	{
		setup_WDT(WDT_2s);
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_enable();
		// BOD禁止処理
		MCUCR |= (1<<BODSE) | (1<< BODS);
		MCUCR = (MCUCR & ~(1 << BODSE))|(1 << BODS);
		sleep_cpu();
		sleep_disable();
		//sleep_mode();
		meas();
		disable_adc();
	}
}
