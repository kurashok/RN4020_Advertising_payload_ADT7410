/*
 * eeprom.c
 *
 * Created: 2021/05/14 11:26:01
 *  Author: kuras
 */ 
#include <avr/io.h>
#include "eeprom.h"

unsigned char read_eeprom(uint16_t adrs)
{
	while((EECR & (1<<EEPE)));
	EEARH = adrs >> 8;
	EEARL = adrs & 0xff;
	EECR = (1<<EERE);
	return EEDR;
}

void write_eeprom(unsigned int uiAddress, unsigned char ucData)
{
	while(EECR & (1<<EEPE)); /* 以前のEEPROMﾌﾟﾛｸﾞﾗﾐﾝｸﾞ完了まで待機 */
	EECR = (0<<EEPM1)|(0<<EEPM0); /* 対応ﾌﾟﾛｸﾞﾗﾐﾝｸﾞ種別設定 */
	EEAR = uiAddress; /* EEPROMｱﾄﾞﾚｽ設定 */
	EEDR = ucData; /* EEPROM書き込み値を設定 */
	EECR |= (1<<EEMPE); /* EEPROM主ﾌﾟﾛｸﾞﾗﾑ許可 */
	EECR |= (1<<EEPE); /* EEPROMﾌﾟﾛｸﾞﾗﾐﾝｸﾞ開始 */
}
