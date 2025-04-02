// StepperTestMain.c
// Runs on TM4C123
// Test the functions provided by Stepper.c,
// 
// Before connecting a real stepper motor, remember to put the
// proper amount of delay between each CW() or CCW() step.
// Daniel Valvano
// September 12, 2013
// Modified by Dr. Min He April 2, 2024

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 4.1, Programs 4.4, 4.5, and 4.6
   Hardware circuit diagram Figure 4.27

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

//LEFT MOTOR
// PD3 connected to driver for stepper motor coil A1
// PD2 connected to driver for stepper motor coil A1'
// PD1 connected to driver for stepper motor coil B1
// PD0 connected to driver for stepper motor coil B1'

//RIGHT MOTOR
// PE3 connected to driver for stepper motor coil A2
// PE2 connected to driver for stepper motor coil A2'
// PE1 connected to driver for stepper motor coil B2
// PE0 connected to driver for stepper motor coil B2'

#include <stdint.h>
#include "stepper.h"
#include "systick.h"
#include "tm4c123gh6pm.h"
#include <stdbool.h>

#define T1ms 			16000U    // Systick reload value to generate 1ms delay, assumes using 16 MHz Clock.
#define HIGHEST_SPEED 		2*T1ms  // fastest speed the stepper can move
#define FASTER_SPEED		5*T1ms
#define STANDARD_SPEED		10*T1ms  // stepper motor speed: output every 10ms, frequency for the stepper motor is 100Hz.
#define HALF_SEC      		500*T1ms //systick reload value for 0.5s

#define BUTTONS          	(*((volatile uint32_t *)0x40025044))  // PORT F, pin:1 & 4
#define BUTTON1MASK   		0x10
#define BUTTON2MASK   		0x01

#define SENSOR 			(*((volatile uint32_t *) 0x40007100)) 

bool too_close = false;

int main(void){
  uint16_t i=0;
	
	//Sensor_Init();
  SysTick_Init();
  Stepper_Init();
  PORTF_Init();
	
  while(1){
		too_close = false;
		if(!(BUTTONS & BUTTON1MASK)){
			Step_Forward(FASTER_SPEED,360);   
			stop_Car();
			SysTick_Wait(HALF_SEC);  // wait for 0.5s
		
		// car turn left 90 degrees: left wheel move backward, right wheel move forward
			Step_Left(FASTER_SPEED,90);   
			stop_Car();
			SysTick_Wait(HALF_SEC);  // wait for 0.5s
			Sensor_Init();
			
			while(!(too_close)){
				step_Car(FASTER_SPEED);
			}
			stop_Car();
			SysTick_Wait(HALF_SEC);
			for(int i = 0; i < 19; ++i){
				SysTick_Wait(HALF_SEC);
			}
				Step_Forward(FASTER_SPEED,600); 
			
		} else if (!(BUTTONS & BUTTON2MASK)){
				Step_Backward(FASTER_SPEED,720);
				stop_Car();	
				SysTick_Wait(HALF_SEC);
				Step_Right(FASTER_SPEED,90);
				stop_Car();	
				SysTick_Wait(HALF_SEC);
				Step_Forward(FASTER_SPEED,270); 
			
		} else {
			stop_Car();
			}
		}
	}

void GPIOPortD_Handler(void)
{
	GPIO_PORTD_ICR_R = 0x40; //acknowledge flag6
	if (SENSOR) {
		too_close = true;
	}else{
		too_close = false;
	}
}

