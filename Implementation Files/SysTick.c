// SysTick.h
// Runs on TM4C123
// Provide functions that initialize the SysTick module and generate 
// specified number of machine cycles delay using busy waiting approach
// Min He, 4/14/2023

#include <stdint.h>
#include "tm4c123gh6pm.h"

// Initialize SysTick with core clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC; // enable SysTick with core clock
}

// Time delay using busy wait.
// The delay parameter is in units of the core clock:1/16MHz 
void SysTick_Wait(uint32_t delay){

	NVIC_ST_RELOAD_R = delay - 1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
  
  while(!(NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT)){};
	
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
}
