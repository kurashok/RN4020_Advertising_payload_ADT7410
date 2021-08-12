/*
 * eeprom.h
 *
 * Created: 2021/05/14 11:27:23
 *  Author: kuras
 */ 
unsigned char read_eeprom(uint16_t adrs);
void write_eeprom(unsigned int uiAddress, unsigned char ucData);
