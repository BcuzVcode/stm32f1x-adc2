#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>

volatile uint16_t adc_value_register;

void ADC_Init(void);
uint16_t ADC_Read(void);

void ADC_Init(void){

	// 1. Enabling clock for GPIO and ADC 
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;		// ENABLE GPIO CLOCK
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		// ENABLE ADC1 CLOCK
	
	// 2. Configure PA0 as Analog input (ADC1 Channel 0)
	GPIOA->CRL &= ~GPIO_CRL_CNF0;			// CLEAR BITS FOR ANALOG MODE
	GPIOA->CRL &= ~GPIO_CRL_MODE0;		// MODE == 00 (INPUT MODE)
	
	// 3. Configure ADC1
	ADC1->CR2 &= ~ADC_CR2_EXTSEL;
	ADC1->CR2 |= (0b111 << 17);
	
	ADC1->CR2 |= ADC_CR2_ADON;				// ENABLE ADC1
	ADC1->CR2 |= ADC_CR2_CAL;					// START ADC1 CALIBRATION
	
	while(ADC1->CR2 & ADC_CR2_CAL);
	
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
	ADC1->SMPR2 |= (0b111 << 0);
	
}

uint16_t ADC_Read(void){

	ADC1->SQR3 = 0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	
	while (!(ADC1->SR & ADC_SR_EOC));
	
	adc_value_register = ((ADC1->DR) & 0x0FFF);
	
	return adc_value_register;
}

int main(void){
	
	ADC_Init();
		
	while(1){
		ADC_Read();
		uint32_t delay_ms = 1000;
		while(delay_ms--){
			__NOP();
		}
	}
	
}