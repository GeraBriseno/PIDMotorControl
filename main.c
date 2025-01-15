#include "stm32f401xe.h"
#include "sysClockConfig.h"
#include "TIM2setUp.h"
#include "TIM3setUp.h"
#include "TIM5setUp.h"
#include "gpioConfig.h"
#include "ADC1Config.h"
#include "I2CIF.h"
#include "MPU6050IF.h"
#include <math.h>
#include "stdio.h"

static uint8_t adcFlag = 0;
static uint8_t mpuFlag = 1;

// Variable to store output of potentiometer after being normalized to 0-45 to simulate accelerometer angle
static uint32_t simulated_angle = 0;

// Variable to store output of potentiometer after being normalized to 0-45 to establish set point for PID controller
static uint32_t set_point = 0;

// Variable to store the difference between setpoint and simulated angle
static int32_t difference = 0;

// Variable to store the average from moving average function for error
static double average_error = 0;

// Variable to store the average from moving average function for pid output
static double average = 0;

// Variable to store the average from moving average function
static double pid_output = 0;

// Variable to store the slope used for normalization of potentiometer ADC value for angle_simulation
static double slope_angle = 1.0 * 45/4095;

// Variable to store the slope used for normalization of pid_output
static double slope_pwm = 1.0 * (200/50);

// Variable to store the pwm applied to the motor
static double applied_pwm = 0;

// Function used to round value obtained from normalization of potentiometer ADC value
static uint32_t roundFunction(double d){
    return floor(d + 0.5);
}

static uint16_t ADC_VAL[2] = {0,0};

static double kp = 1;
static double kd = 1;
static double ki = .1;
static double sampling_rate = 0.005;
#define MOVING_AVERAGE_BUFFER_SIZE 100
static double moving_average_buffer[MOVING_AVERAGE_BUFFER_SIZE];
static double moving_average_buffer_error[MOVING_AVERAGE_BUFFER_SIZE];
static double sum = 0;
static double* pSum = &sum;
static double sumPID = 0;
static double* pSumPID = &sumPID;

static double movingAverageFunction(double new_angle, double *buffer, double *pSumArg){
	static double averaged_angle;
	
	for(int i = MOVING_AVERAGE_BUFFER_SIZE-1; i > -1; i--){
		if(i != 0){
			buffer[i] = buffer[i-1];
			}
		else{
			buffer[i] = new_angle; 
			}
	}
	
	for(int i = 0; i < MOVING_AVERAGE_BUFFER_SIZE; i++){
			*pSumArg += buffer[i];
    }
	
	averaged_angle = *pSum/MOVING_AVERAGE_BUFFER_SIZE;

	*pSumArg = 0;
		
	return averaged_angle;
}

static double PID_controller(double error, double *pSumPIDArg){
	
	static double old_error, pid_output;
	
	*pSumPIDArg = *pSumPIDArg + error;
	
	pid_output = (kp*error) + (ki * (*pSumPIDArg) * sampling_rate) + (kd * ((error - old_error)/sampling_rate));
	
	old_error = error;

	return pid_output;
}

void TIM3_IRQHandler(void)
{
  if(TIM3->SR & TIM_SR_UIF)
  {	
		
    TIM3->SR &= ~TIM_SR_UIF;
  }
}

void TIM5_IRQHandler(void)
{
  if(TIM5->SR & TIM_SR_UIF)
  {	
		if(adcFlag == 1){

			MPU6050_Read_Accel();
			average_error = movingAverageFunction(difference, moving_average_buffer_error, pSum);
			pid_output = PID_controller(average_error, pSumPID);
			average = movingAverageFunction(pid_output, moving_average_buffer, pSum);
			applied_pwm = roundFunction(slope_pwm * average)+1010;
			
			if(applied_pwm > 1200){
				applied_pwm = 1200;
			}
			
			TIM2->CCR1 = applied_pwm;
			
			adcFlag = 0;
			mpuFlag = 1;
		}
		TIM5->SR &= ~TIM_SR_UIF;
  }
}

// Our main function
int main (void){
	
	// Run our configuration functions
	SysClockConfig();
	configureTIM2_PWM();
	configureTIM3();
	configureTIM5();
	GPIO_Config();
	ADC_Init();
	ADC_Enable();
	
	I2C_Config();	
	MPU6050_Init();
	
	// Our while loop
	while (1)
	{	
		if(mpuFlag == 1){
			// Read value from ADC channel 1
			ADC_Start(1);
			ADC_WaitForConv();
			ADC_VAL[0] = ADC_GetVal();
			
			//Normalize ADC1 channel 1 input to a 0-45 scale
			simulated_angle = roundFunction(slope_angle * ADC_VAL[0]);
			
			// Read value from ADC channel 4
			ADC_Start(4);
			ADC_WaitForConv();
			ADC_VAL[1] = ADC_GetVal();
			
			//Normalize ADC1 channel 4 input to a 0-45 scale
			set_point = roundFunction(slope_angle * ADC_VAL[1]);
			
			difference = set_point - simulated_angle;
			
			mpuFlag = 0;
			adcFlag = 1;
		}
	}
	
}
