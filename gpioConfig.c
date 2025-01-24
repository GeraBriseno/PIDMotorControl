#include "gpioConfig.h"
#include "sysClockConfig.h"

//Function to configure GPIO port and pins used, in this case GPIO port A
void GPIO_Config(void){
	
	// 1. Enable the GPIOA and GPIOB CLOCKS
	RCC->AHB1ENR |= (1<<0);
	RCC->AHB1ENR |= (1<<1);
	
	//Configure the OUTPUT MODE FOR GPIOA
	GPIOA->OTYPER = 0;
	GPIOA->OSPEEDR = 0;
	
	//Set the respective GPIOB pins' mode as open drain output
	GPIOB->OTYPER |= (1<<8);
	GPIOB->OTYPER |= (1<<9);
	
	//Set the Respective GPIOA PINs to Analog Mode	
	GPIOA->MODER |= (3<<8);  // analog mode for PA 4 (channel 4)	
	GPIOA->MODER |= (3<<2);  // analog mode for PA 1 (channel 1)
	
	//Set the respective GPIOA pins as Alternate Mode
	GPIOA->MODER |= (2<<0);  // alternate mode for PA 0
	
	//Set the respective GPIOB pins as Alternate Mode
	GPIOB->MODER |= (2<<16);  // alternate mode for PB 8
	GPIOB->MODER |= (2<<18);  // alternate mode for PB 9
	
	//Set the respective GPIO pins' Alternate Function
	GPIOA->AFR[0] |= (1<<0);  //set alternate function as af1 (TIM2_CHANNEL_1)
	
	//Set the respective GPIOB pins' Alternate Function
	GPIOB->AFR[1] |= (4<<0);  //set alternate function of GPIOB 8 as af4 (I2C1 SCL)
	GPIOB->AFR[1] |= (4<<4);   //set alternate function of GPIOB 9 as af4 (I2C1 SDA)
	
	//Set the respective GPIOB pins' speed as very high
	GPIOB->OSPEEDR |= (3<<16);
	GPIOB->OSPEEDR |= (3<<18);
	
	//Set the respective GPIOB pins as pull up
	GPIOB->PUPDR |= (1<<16);
	GPIOB->PUPDR |= (1<<18);
	
}
