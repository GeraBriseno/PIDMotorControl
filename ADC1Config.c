#include "sysClockConfig.h"
#include "ADC1Config.h"

//Function to initialize ADC
void ADC_Init(void){
	
	//1. Enable ADC1 clock
	RCC->APB2ENR |= (1<<8);  // enable ADC1 clock
	
	//2. Set the prescaler in the ADC Common Control Register
	ADC->CCR |= 1<<16;  		 // PCLK2 divided by 4 (80/4)
		
	//3. Set the scan mode and resolution in the Control Register 1
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 |= (0<<24);   // 12 bit resolution
		
	//4. Set the continuous conversion, EOC (end of conversion), and data alignment in Control Register 2 (CR2)
	ADC1->CR2 |= (1<<1);     // Enable continuous conversion mode
	ADC1->CR2 |= (1<<10);    // EOCS after each conversion
	ADC1->CR2 |= (0<<11);    // Data alignment right
		
	//5. Set the sampling time for the channels	
	ADC1->SMPR2 |= (0<<12);  // Sampling time of 3 cycles for channel 4
	ADC1->SMPR2 |= (0<<3);  // Sampling time of 3 cycles for channel 1

	//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (1<<20);   // SQR1_L = 1 for 2 conversions, that is two channels
		
}


//Function to turn ADC on
void ADC_Enable(void){
	
	//Turn ADC converter on
	ADC1->CR2 |= 1<<0;
	
	//Wait for ADC to stabilize (approx 10us)
	uint32_t delay = 10000;
	while (delay--);
	
}


// Function to start ADC conversion
void ADC_Start(int channel){

	//Clear conversion sequence
	ADC1->SQR3 = 0;
	
	//Set channel in the conversion sequence
	ADC1->SQR3 |= (channel<<0);
	
	//Clear status register
	ADC1->SR = 0;
	
	//Start conversion, by setting SWSTART bit in CR2
	ADC1->CR2 |= (1<<30);
}


// Function to wait for ADC conversion
void ADC_WaitForConv(void){
	
	//Wait for EOC (end of conversion) flag to be set
	while (!(ADC1->SR & (1<<1)));
	
}


// Function to get ADC value from ADC1 data register
uint16_t ADC_GetVal(void){
	
	// Read the data register
	return ADC1->DR;
}