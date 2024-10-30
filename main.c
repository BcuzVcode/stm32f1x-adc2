#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>

volatile uint16_t adc_value_register;


void Delay_ms(uint16_t ms) {
    while(ms--){__NOP();}
}

void SysTick_Init(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);  // 1ms ticks
}

void Clock_Init(void) {
    // Reset clock configuration to default state
    RCC->CR |= RCC_CR_HSION;                     // Enable HSI
    while(!(RCC->CR & RCC_CR_HSIRDY));           // Wait for HSI ready
    
    // Reset CFGR register
    RCC->CFGR = 0x00000000;
    
    // Disable PLL
    RCC->CR &= ~RCC_CR_PLLON;
    
    // Select HSI as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;                   // HSI as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0);     // Wait for HSI
}

void ADC_Init(void) {
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;     // Enable GPIO clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // Enable ADC1 clock
    
    // Set ADC prescaler
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;          // Clear prescaler bits
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV2;      // Set ADC prescaler to /2
    
    // Configure PA0 as analog input
    GPIOA->CRL &= ~GPIO_CRL_CNF0;           // Clear CNF bits for analog mode
    GPIOA->CRL &= ~GPIO_CRL_MODE0;          // Clear MODE bits for input mode
    
    // Power up ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    for(uint16_t i = 0; i < 100; i++) __NOP();
    
    // Calibrate ADC
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);
    
    // Configure for single conversion
    ADC1->CR2 &= ~ADC_CR2_CONT;             // Single conversion mode
    ADC1->CR2 &= ~ADC_CR2_EXTTRIG;          // Software trigger
    
    // Set sampling time
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
    ADC1->SMPR2 |= (0b100 << 0);            // Set to 41.5 cycles
}

uint16_t ADC_Read(void) {
    ADC1->SQR3 = 0;                         // Select channel 0
    ADC1->CR2 |= ADC_CR2_SWSTART;          // Start conversion
    
    while(!(ADC1->SR & ADC_SR_EOC));       // Wait for conversion
    
    adc_value_register = (ADC1->DR & 0x0FFF);
    
    return adc_value_register;
}

int main(void) {
    Clock_Init();
    SysTick_Init();
		//SystemInit();
    ADC_Init();
    
    while(1) {
        adc_value_register = ADC_Read();
        Delay_ms(1000);
    }
}