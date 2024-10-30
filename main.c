#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>

volatile uint16_t adc_value_register;
volatile uint32_t msTicks = 0;  // Counter for milliseconds

void SysTick_Handler(void) {
    msTicks++;
}

void Delay_ms(uint32_t ms) {
    uint32_t startTicks = msTicks;
    while((msTicks - startTicks) < ms);
}

void SysTick_Init(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);  // Configure for 1ms ticks
}

void ADC_Init(void) {
    // 1. Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;     // Enable GPIO clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // Enable ADC1 clock
    
    // Set ADC prescaler
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;          // Clear prescaler bits
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;      // Set ADC prescaler to /6
    
    // 2. Configure PA0 as analog input
    GPIOA->CRL &= ~GPIO_CRL_CNF0;           // Clear CNF bits for analog mode
    GPIOA->CRL &= ~GPIO_CRL_MODE0;          // Clear MODE bits for input mode
    
    // 3. Configure ADC1
    ADC1->CR2 &= ~ADC_CR2_EXTTRIG;          // Disable external trigger
    ADC1->CR2 |= ADC_CR2_CONT;              // Enable continuous conversion mode
    
    // First ADC enable (wake up)
    ADC1->CR2 |= ADC_CR2_ADON;
    for(uint16_t i = 0; i < 100; i++) __NOP();  // Short delay
    
    // Start calibration
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);         // Wait for calibration to complete
    
    // Configure sample time (max for channel 0)
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
    ADC1->SMPR2 |= (0b111 << 0);            // 239.5 cycles
    
    // Second ADC enable (for conversion)
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC_Read(void) {
    ADC1->SQR3 = 0;                         // Select channel 0
    ADC1->CR2 |= ADC_CR2_SWSTART;          // Start conversion
    
    while(!(ADC1->SR & ADC_SR_EOC));       // Wait for conversion to complete
    
    adc_value_register = (ADC1->DR & 0x0FFF);  // Read and mask to 12 bits
    
    return adc_value_register;
}

int main(void) {
    SysTick_Init();  // Initialize SysTick for timing
    ADC_Init();
    
    while(1) {
        adc_value_register = ADC_Read();
        Delay_ms(1000);  // Wait for 1 second
    }
}