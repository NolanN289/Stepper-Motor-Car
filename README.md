# Description:

This is a stepper motor car. It is drive by a TM4C123 Microcontroller from Texas Instruments and programmed in embedded C using the Keil uVision IDE. It is driven by a power supply circuit which 6V is applied from a AA battery pack. For the stepper motor, the classic 28BYJ-48 5V DC Motor. To supply the necessary current to the stepper motors, two ULN2003 motor drivers are also used. To detect an obstacle in front of the car, an IR sensor module was used to perform this task.

# Construction:

To build the actual car, the mounts and wheels were 3D printed and screwed onto the chassis with M3 screws. The mount was printed for the specific stepper motor in mind. The STL files for the mount and wheels are included, and they were found from Thingverse. In addition, the two motor drivers were also screwed into the chassis. The microcontroller was tied to the chassis to make debugging the car itself easy and accessible. The IR sensor was also screwed to the car chassis to ensure mechanical integrity and consistent of the sensor itself. 

Here is what it ended up looking like:

![image](https://github.com/NolanN289/Stepper-Motor-Car/assets/174823448/82944515-3e86-4a00-a6b6-4046cfe8c99a)

# Implementation:

To implement the car, we need to initialize the proper ports for the GPIO we wish to use. To determine which ports to use, bit specific addressing is used to assign values to the switches and IR sensor. SysTick is implemented to generate an accurate 0.5s delay for the operation of the code. The system uses a Finite State Machine to implement the sequence of sending pulses to the stepper motor to make it move properly. It also uses interrupts to interface the IR sensor to detect an object and stop the car as soon as an object is detected. We used a priority 3 interrupt to implement this into the car.

# Schematic of the Car:

![image](https://github.com/NolanN289/Stepper-Motor-Car/assets/174823448/ea007aeb-f51e-422a-aea3-e8baec74a5eb)

Note: Done through Scheme-It on DigiKey
