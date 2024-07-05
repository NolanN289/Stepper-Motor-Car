// Stepper.h
// Runs on LM4F120/TM4C123
// Provide functions that step the motor and initialize the stepper motor interface.
// Daniel Valvano
// September 12, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Example 4.1, Programs 4.4, 4.5, and 4.6
   Hardware circuit diagram Figure 4.27

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

 
#include <stdint.h>

// Initialize Stepper interface
void Stepper_Init(void);
void PORTF_Init(void);
void Sensor_Init(void);
void stop_Car(void);
void step_Car(uint32_t speed);
void Step_Right(uint32_t speed, uint16_t degree);
void Step_Left(uint32_t speed, uint16_t degree);
void Step_Backward(uint32_t speed, uint16_t degree);
void Step_Forward(uint32_t speed, uint16_t degree);

