//******************** (C) Yifeng ZHU ********************
// @file    main.c
// @author  Yifeng Zhu
// @version V1.0.0
// @date    November-11-2012
// @note    
// @brief   C code for STM32L1xx Discovery Kit
// @note
//          This code is for the book "Embedded Systems with ARM Cortex-M3 
//          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
//          ISBN-10: 0982692625.
// @attension
//          This code is provided for education purpose. The author shall not be 
//          held liable for any direct, indirect or consequential damages, for any 
//          reason whatever. More information can be found from book website: 
//          http://www.eece.maine.edu/~zhu/book
//********************************************************

#include <stdint.h>

/* Standard STM32L1xxx driver headers */
#include "stm32l1xx.h"

/* STM32L1xx Discovery Kit:
    - USER Pushbutton: connected to PA0 (GPIO Port A, PIN 0), CLK RCC_AHBENR_GPIOAEN
    - RESET Pushbutton: connected RESET
    - GREEN LED: connected to PB7 (GPIO Port B, PIN 7), CLK RCC_AHBENR_GPIOBEN 
    - BLUE LED: connected to PB6 (GPIO Port B, PIN 6), CLK RCC_AHBENR_GPIOBEN
    - Linear touch sensor/touchkeys: PA6, PA7 (group 2),  PC4, PC5 (group 9),  PB0, PB1 (group 3)
*/



//******************************************************************************************
//* The main program starts here
//******************************************************************************************



#include <stdint.h>

/* Standard STM32L1xxx driver headers */
#include "stm32l1xx.h"

/* STM32L1xx Discovery Kit:
    - USER Pushbutton: connected to PA0 (GPIO Port A, PIN 0), CLK RCC_AHBENR_GPIOAEN
    - RESET Pushbutton: connected RESET
    - GREEN LED: connected to PB7 (GPIO Port B, PIN 7), CLK RCC_AHBENR_GPIOBEN 
    - BLUE LED: connected to PB6 (GPIO Port B, PIN 6), CLK RCC_AHBENR_GPIOBEN
    - Linear touch sensor/touchkeys: PA6, PA7 (group 2),  PC4, PC5 (group 9),  PB0, PB1 (group 3)
*/
#define Auto_reload 200 					// Auto-reload value register ARR

uint16_t TimingDelay;

// Enable the clock of GPIO port x
void GPIO_Clock_Enable(int shiftbit){
	// Select enable the clock to GPIO Port 	
	RCC->AHBENR		|= 0x01<< shiftbit; 
}

// Enable the clock of timer x
void Timer_Clock_Enable(int shiftbit){
	// Select timer enable the clock
	RCC->APB1ENR |= 0x01<< shiftbit;
}

void GPIOB_Pin_Led_Init(uint8_t numled){
	// Set pin I/O mode as alternative function
	// 00 = digital input(default)		01 = digital ouput
	// 10 = alternative function			11 = analog
	GPIOB->MODER &= ~(0x03<<(2*numled));  // Mode mask 
	GPIOB->MODER |= 0x02<<(2*numled);
	// Pin as alternative function low (TIM4)
	GPIOB->AFR[0] |= 0x2 << (4*numled);
	
	// Output type puss-pull 0, open-drain 1
	GPIOB->OTYPER &= ~(0x01<<numled);
	
	// Set IO Output speed; 
	// 00 = 400 Khz, 01 = 2Mhz, 10 = 10Mhz, 11 = 40Mhz
	GPIOB->OSPEEDR &= ~(0x03<<(2*numled));  // Speed mask
	GPIOB->OSPEEDR |= 0x03<<(2*numled); 		// 2Mhz
	
	// Set IO as no pull-up pull-down
	// 00 = no pull-up, no pull-down
	// 01 = pull-up, 10 = pull-down, 11 = reverse
	GPIOB->PUPDR &= ~(0x03<<(2*numled));
}

void TIM4_Channel1_Init(uint8_t hex){
	TIM4->PSC = 63; // Prescaler = 63
	
	// Auto-reload: upcounting (0->ARR), downcounting (ARR->0)
	TIM4->ARR = Auto_reload - 1;
	
	// OC1M = 110 for PWM Mode 1 output on channel 1
	TIM4->CCMR1 |= hex<<4;
	
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE; // Output 1 preload enable
	
	TIM4->CR1 |= TIM_CR1_ARPE;			// Auto-reload preload enable
	TIM4->CCER |= TIM_CCER_CC1E;		// Enable output for channel 1
	TIM4->EGR |= TIM_EGR_UG;				// Force update
	TIM4->SR &= ~TIM_SR_UIF;				// Clear the update flag
	TIM4->DIER |= TIM_DIER_UIE;			// Enable interrupt on update event
	TIM4->CR1 |= TIM_CR1_CEN;				// Enable counter
	
}

void TIM4_Channel2_Init(uint16_t hex){
	TIM4->PSC = 63; // Prescaler = 63
	
	// Auto-reload: upcounting (0->ARR), downcounting (ARR->0)
	TIM4->ARR = Auto_reload - 1;
	
	// OC2M = 111 for PWM Mode 2 output on chanel 2
	TIM4->CCMR1 |= hex<<12;
	
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;	// Output 2 preload enable
	
	TIM4->CR1 |= TIM_CR1_ARPE;			// Auto-reload preload enable
	TIM4->CCER |= TIM_CCER_CC2E;		// Enable output for channel 2
	TIM4->EGR |= TIM_EGR_UG;				// Force update
	TIM4->SR &= ~TIM_SR_UIF;				// Clear the update flag
	TIM4->DIER |= TIM_DIER_UIE;			// Enable interrupt on update event
	TIM4->CR1 |= TIM_CR1_CEN;				// Enable counter
	
}

void SysTick_MSI_Clock_Config(int shiftbit){
	// Setting MSIRANGE bits of RCC_ICSCR
	RCC->ICSCR &= ~(0x07<<shiftbit);
	RCC->ICSCR |= 0x06<<shiftbit;
	// Internal Multi Speed clock enable 
	RCC->CR |= RCC_CR_MSION;
	// Wait for internal Multi Speed clock ready flag
	while(!(RCC->CR & RCC_CR_MSIRDY));	
}

void SysTick_Initialize(uint32_t counts){
	// Disable SysTick IRQ and SysTick counter
	SysTick->CTRL = 0;
	
	// Set reload register
	SysTick->LOAD = counts - 1;
	
	// Set priority
	NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	
	// Reset the SysTick counter value
	SysTick->VAL = 0;
	
	// Select processcer clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE;
	
	// Enable SysTick IRQ and SysTick timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	
	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 1 = counting down to zero does not asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT;	
}

void SysTick_Handler(void){
	if (TimingDelay != 0) TimingDelay --;
}

void Delay(uint32_t nTime){
	TimingDelay = nTime;
	while(TimingDelay != 0);
}


int main(void){
	int brightness = 1;		// Set brightness
	int stepSize = 1;			// Direction
	
	// The AHB clock frequency as 4.194 MHz
	//SysTick_MSI_Clock_Config(13);
	// Enable systick
	SysTick_Initialize(2097);
	// Enable GPIO port B
	GPIO_Clock_Enable(1);
	// Enable Timer 4
	Timer_Clock_Enable(2);
	// Mode 1 channel 1
	TIM4_Channel1_Init(0x06);
	// Mode 2 channel 2
	TIM4_Channel2_Init(0x07);
	// PB.6
	GPIOB_Pin_Led_Init(6);
	// PB.7
	GPIOB_Pin_Led_Init(7);
	
	while(1){
		if((brightness >= 200) || (brightness <=0))
			stepSize = -stepSize;			// Reverse direction
		brightness += stepSize;
		TIM4->CCR1 = brightness;		// Set brightness for chanel 1
		TIM4->CCR2 = brightness;		// Set brightness for channel 2
		// short delay
		Delay(10);								// Delay 10ms
	}
}





