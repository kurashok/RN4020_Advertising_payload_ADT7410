/*
 * adc.h
 *
 * Created: 2021/01/29 12:13:38
 *  Author: kuras
 */ 


#ifndef ADC_H_
#define ADC_H_

void setup_adc(int ch);
void disable_adc();
void enable_adc();
uint16_t exec_adc();

#endif /* ADC_H_ */