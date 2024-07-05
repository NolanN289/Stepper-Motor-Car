// Stepper.c
// Runs on TM4C123
// Starter file for Stepper Motor Car
// by Nolan Nguyen

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
#include "tm4c123gh6pm.h"
#include "systick.h"
#include <stdbool.h>

struct State{
  uint8_t Left;     // Output
  uint8_t Right;     // Output
  uint8_t Next[2]; // CW/CCW Next[0]->CW, Next[1]->CCW
};
typedef const struct State StateType;

// finish the following constant definitions
#define FORWARD 1        // Next index
#define BACKWARD 0       // Next index
#define HUNDRED_STEPS 18       // each step moves 0.18 degree, 100 steps moves 18 degree

enum states {S0, S1, S2, S3};


// finish the FSM definition
StateType fsm[4]={
	// index 0: state 0,state goes from 0 to 3,
	// to move forward: PD3-0:left CCW; PE3-0,right CW
	// CW state transition is: 3->6->12->9 then repeat
	// CCW state transition is: 3->9->12->6 then repeat
	{ 3,3,  {S1,S3}}, // state 0,  PD3-0:0011 CW next state->1, CCW next state->3
  { 6,9, {S2,S0}}, // state 1,  PD3-0:0110
  { 12,12, {S3,S1}}, // state 2,  PD3-0:1100
  { 9,6,  {S0,S2}}, // state 3,  PD3-0:1001
};

uint8_t s, l_s, r_s; // current state

// define bit addresses for stepper motor pins
#define LEFT_STEPPER  		(*((volatile uint32_t *)0x4000703C))  // PORT D, pin: 0,1,2,3
#define RIGHT_STEPPER  		(*((volatile uint32_t *)0x4002403C))  // PORT E, pin: 0,1,2,3
#define BUTTON          	(*((volatile uint32_t *)0x40025040))  // PORT F, pin: 4
	

#define PORT_D_MASK 0x0F						// PD0-3
#define PORT_D_PCTLMASK 0x0000FFFF

#define CLEARPDMASK   0x0F000000  //PD6
#define PORTDMASK     0x40

#define PORT_E_MASK 0x0F
#define PORT_E_PCTLMASK 0x0000FFFF

#define PORTF_MASK 0x11
#define PORT_F_PCTLMASK  0x000F000F
 
// stop the stepper motor car and reset FSM states
void stop_Car(void) {
	
	  // all outputs are 0
		LEFT_STEPPER = 0x00;  // Clear all bits controlling the left stepper
    RIGHT_STEPPER = 0x00; // Clear all bits controlling the right stepper

    // Reset the current state indices to initial state 
    l_s = 0;
    r_s = 0;
    s = 0;
}

void step_Car(uint32_t speed){
	LEFT_STEPPER = fsm[s].Left; // step motor
  RIGHT_STEPPER = fsm[s].Right; 
  SysTick_Wait(speed);
  s = fsm[s].Next[FORWARD]; // clock wise circular
	
}

// Move forward, speed is the systick reload value to control time to wait after each step
// each step moves 0.18 degree: one complete circle is 360 degrees
void Step_Forward(uint32_t speed, uint16_t degree){
	uint16_t i,n=(100 *degree)/HUNDRED_STEPS;
	
		for (i=0;i<n;i++) {
        LEFT_STEPPER = fsm[s].Left; // step motor
        RIGHT_STEPPER = fsm[s].Right; 
        SysTick_Wait(speed);
        s = fsm[s].Next[FORWARD]; // clock wise circular
        //r_s = fsm[s].Next[s]; // clock wise circular
    }

}

// Move backward, speed is the systick reload value to control time to wait after each step
// each step moves 0.18 degree: one complete circle is 360 degrees
void Step_Backward(uint32_t speed, uint16_t degree){
	uint16_t i,n=(100*degree)/HUNDRED_STEPS;
	
		for (i=0;i<n;i++) {
		LEFT_STEPPER = fsm[s].Left; // step motor
		RIGHT_STEPPER = fsm[s].Right;
		SysTick_Wait(speed);
		s = fsm[s].Next[BACKWARD]; // clock wise circular
		//r_s = fsm[s].Next[FORWARD]; // clock wise circular
	}
		
}

// car turn right: left wheel move forward(CCW), right wheel move backward(CCW)
void Step_Right(uint32_t speed, uint16_t degree){	
	uint16_t i,n=(200*degree)/HUNDRED_STEPS;
	
		for (i=0;i<n;i++) {
		LEFT_STEPPER = fsm[l_s].Left; // step motor
		RIGHT_STEPPER = fsm[r_s].Right;
		SysTick_Wait(speed);
		l_s = fsm[l_s].Next[FORWARD]; // clock wise circular
		r_s = fsm[r_s].Next[BACKWARD];
	}
}

// car turn left: left wheel move backward(CW), right wheel move forward(CW)
void Step_Left(uint32_t speed, uint16_t degree){	
	uint16_t i,n=(175*degree)/HUNDRED_STEPS;
	
		for (i=0;i<n;i++) {
		LEFT_STEPPER = fsm[l_s].Left; // step motor
		RIGHT_STEPPER = fsm[r_s].Right;
		SysTick_Wait(speed);
		l_s = fsm[l_s].Next[BACKWARD]; // clock wise circular
		r_s = fsm[r_s].Next[FORWARD]; // clock wise circular
	}
}


// Initialize left Stepper motor interface
void Stepper_Init(void){
	if ((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R3)!= SYSCTL_RCGCGPIO_R3){
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;     								// activate D clock
		while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R3)!=SYSCTL_RCGCGPIO_R3){} // wait for the clock to be ready
  }
		s = 0; 
                                    
  GPIO_PORTD_AMSEL_R &= ~PORT_D_MASK;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~PORT_D_PCTLMASK; // 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R |= PORT_D_MASK;   // 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~PORT_D_MASK;// 6) disable alt funct on PD3-0
  GPIO_PORTD_DR8R_R |= PORT_D_MASK;  // enable 8 mA drive
  GPIO_PORTD_DEN_R |= PORT_D_MASK;   // 7) enable digital I/O on PD3-0 
	
	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // 1) activate port E
                                    
  GPIO_PORTE_AMSEL_R &= ~PORT_E_MASK;      // 3) disable analog functionality on PE3-0
  GPIO_PORTE_PCTL_R &= ~PORT_E_PCTLMASK; // 4) GPIO configure PE3-0 as GPIO
  GPIO_PORTE_DIR_R |= PORT_E_MASK;   // 5) make PE3-0 out
  GPIO_PORTE_AFSEL_R &= ~PORT_E_MASK;// 6) disable alt funct on PE3-0
  GPIO_PORTE_DR8R_R |= PORT_E_MASK;  // enable 8 mA drive
  GPIO_PORTE_DEN_R |= PORT_E_MASK;   // 7) enable digital I/O on PE3-0 

}

void Sensor_Init(void) 
{ 
	if ((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R3)!= SYSCTL_RCGCGPIO_R3){
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;     								// activate D clock
		while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R3)!=SYSCTL_RCGCGPIO_R3){} // wait for the clock to be ready
  }
	
	GPIO_PORTD_AMSEL_R &= ~PORTDMASK; 				  						//disable analog function
	GPIO_PORTD_PCTL_R  &= ~CLEARPDMASK;    						// enable regular GPIO
	GPIO_PORTD_DIR_R 	 &= ~PORTDMASK; 					
	GPIO_PORTD_AFSEL_R &= ~PORTDMASK; 				  						//no alternate function
	GPIO_PORTD_DEN_R 	 |= PORTDMASK;						

	GPIO_PORTD_IS_R &= ~PORTDMASK;         											// PD6 is edge-sensitive
  GPIO_PORTD_IBE_R |= PORTDMASK;         											// PD6 both edges trigger
  GPIO_PORTD_ICR_R = PORTDMASK;         									 			// Clear any prior interrupt on PD6
  GPIO_PORTD_IM_R |= PORTDMASK;          											// Enable interrupt on PD6
  NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFF0FFF) | 0x0000C000; 	// Priority 3
  NVIC_EN0_R = 0x00000008;          											// Enable interrupt 3 in NVIC
}


void PORTF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // 1) activate port F
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   	// unlock PortF PF
	GPIO_PORTF_CR_R |= PORTF_MASK;         		// allow changes to PF4 :10000->0x10     
  GPIO_PORTF_AMSEL_R &= ~PORTF_MASK;        // disable analog function
  GPIO_PORTF_PCTL_R &= ~PORT_F_PCTLMASK; 				// GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R &= ~PORTF_MASK;          // PF4 input   
	GPIO_PORTF_AFSEL_R &= ~PORTF_MASK;        // no alternate function
  GPIO_PORTF_PUR_R |= PORTF_MASK;          	// enable pullup resistors on PF4       
  GPIO_PORTF_DEN_R |= PORTF_MASK;          	// enable digital pins PF4
}