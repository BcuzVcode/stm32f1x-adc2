#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>

volatile uint16_t adc_value_register;
volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
    msTicks++;
}

void Delay_ms(uint32_t ms) {
    uint32_t startTicks = msTicks;
    while((msTicks - startTicks) < ms);
}

void SysTick_Init(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock/1000);  // 1ms ticks
}

void Clock_Init(void) {
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;                     // Enable HSE
    while(!(RCC->CR & RCC_CR_HSERDY));           // Wait for HSE ready
    
    // Configure Flash latency for higher frequency
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;           // Two wait states
    
    // Configure PLL (8MHz HSE * 9 = 72MHz)
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (RCC_CFGR_PLLSRC |             // HSE as PLL source
                  RCC_CFGR_PLLMULL9);            // PLL x9
    
    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    
    // Set PLL as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    // Set bus prescalers
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;           // APB1 = HCLK/2 (36MHz max)
}

void ADC_Init(void) {
    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;	// Enable GPIO clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // Enable ADC1 clock
    
    // Set ADC prescaler
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;          // Clear prescaler bits
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;      // Set ADC prescaler to /6
    
    // Configure PA0 as analog input
    GPIOA->CRL &= ~GPIO_CRL_CNF0;           // Clear CNF bits for analog mode
    GPIOA->CRL &= ~GPIO_CRL_MODE0;          // Clear MODE bits for input mode
    
    // Configure ADC1
    ADC1->CR2 &= ~ADC_CR2_EXTTRIG;          // Disable external trigger
    ADC1->CR2 |= ADC_CR2_CONT;              // Enable continuous conversion mode
    
    // First ADC enable (wake up)
    ADC1->CR2 |= ADC_CR2_ADON;
    for(uint16_t i = 0; i < 100; i++) __NOP();  // Short delay
    
    // Start calibration
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);         // Wait for calibration
    
    // Configure sample time
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
    ADC1->SMPR2 |= (0b111 << 0);            // 239.5 cycles
    
    // Second ADC enable
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC_Read(void) {
    ADC1->SQR3 = 0;                         // Select channel 0
    ADC1->CR2 |= ADC_CR2_SWSTART;          // Start conversion
    
    while(!(ADC1->SR & ADC_SR_EOC));       // Wait for conversion
    
    adc_value_register = (ADC1->DR & 0x0FFF);
    
    return adc_value_register;
}

int main(void) {
    //Clock_Init();
    //SysTick_Init();
		SystemInit();
    ADC_Init();
    
    while(1) {
        adc_value_register = ADC_Read();
        Delay_ms(1000);
    }
}