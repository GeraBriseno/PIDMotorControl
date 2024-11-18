#include "TIM2setUp.h"
#include "sysClockConfig.h"

void configureTIM2_PWM(void){
	
	//Enable TIM2 peripheral clock
	RCC->APB1ENR |= (1 << 0);
	
	//Channels 1 configured as output
	TIM2->CCMR1 |= (0<<0);
	
	//Set capture/compare polarity as active low for channel 1
	TIM2->CCER &=~ TIM_CCER_CC1P;  // |= (1<<1)
	
	//Set output compare mode as PWM mode 1 for channel 1
	TIM2->CCMR1 |= (6<<4);
	
	//Set prescaler value for TIM2, to achieve a frequency of 1KHz
	TIM2->PSC = 79;    //Set TIM2 Frequency to 1 MHz (80,000,000/(79+1)) using prescaler
										 //this together with the value for ARR register will give us a resolution of 1 microsecond
										 //for a smoother speed control for our motor
	
	//Set autoreload value for TIM2, this is the period for our PWM output
	TIM2->ARR = (uint32_t)20000;   //50HZ freq, period of 20ms 
	
	//Set value for capture compare register for channels 1 and 2, this will be the duty of our PWM output
	TIM2->CCR1 = 1000;    //Duty of our PWM pulse
	
	//Enable output compare preload for channels 1 and 2
	TIM2->CCMR1 |= (1<<3);
	
	//Enable autoreload preload for TIM2
	TIM2->CR1 |= (1<<7);
	
	//Select edge aligned mode for TIM2, the counter counts up or down depending on the direction bit
	TIM2->CR1 |= (0<<5);
	
	//Set direction bit as upcounter
	TIM2->CR1 |= (0<<4);
	
	//Enable capture/compare signal as output in corresponding channel pins
	TIM2->CCER |= (1<<0);
	
	//Enable TIM2 counter
	TIM2->CR1 |= (1<<0);
}