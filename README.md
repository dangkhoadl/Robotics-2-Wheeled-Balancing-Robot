# 2-Wheel Balancing Robot

## Hardwares Resources
* 1 STM32F4 Microcontroller development board (ARM Cortex M4)
	- 2 PWM channels
	- 4 timer interrupts
	- 2 external event interrupts
	- 1 SPI channel
	- 1 UART channel
* 2 DC motors with PWM control
	- 1 motor to control 1 wheel (left and right)
* 2 infrared encoder
	- 1 infrared encoder to measure the speed and position of each wheel
* 1 MPU-6050 Gyroscope
	- 10-axis gyroscope
	- Interface with the microcontroller through SPI
* 1 Xbox360 Controller
	- Interface with Microcontroller through a wireless module

## Features
* PID Controller for balancing
	- Implemented in the ARM CPU
	- Realtime response to different weight load with 2 nested loop PID algorithm
	- Input data: 
		+ From 10 different gyroscope channels
		+ Form 2 wheel encoders
	- Output: PWM signals to control wheel movement and balance the robot 
* Kalman filter
	- To eliminate Input noises from input data

## Demo
* [Movements](https://www.youtube.com/watch?v=6Qq1v7zkcBc)
* [Heavy-weight payload test](https://www.youtube.com/watch?v=mP4tnT0PIg8)

## How to run
* Use [ARM Keil IDE ](http://www2.keil.com/mdk5/)  to build and compile project files into .bin object
* Use the built-in debugger to load the object file into the chip ROM
