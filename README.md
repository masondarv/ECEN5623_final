# Final Project for ECEN5623 - Real-Time Embedded Systems

# Virtual Race Car Controller

## Overview

This project implements a real-time multi-threaded application for autonomously driving a car around a race track in the video game Forza Motorsport.

It consists of 4 processing threads:

* __Sequencer thread__ - Controls the timing of all other processing threads through semaphores.
  
* __Image capture thread__ - Captures a photo of the monitor screen, and performs some image pre-processing.
  
* __Image processing thread__ - Uses openCV2 functions to determine the linear position and angle of the car relative to the optimal path displayed by the game, represented with a blue arrow.
  
* __Controller thread__ - Implements a PID controller to regulate steering, throttle and braking. Sends commands to Xbox 360 controller emulator via UART.

This application was developed for an embedded Linux environment. This application was tested using an NVIDIA Jetson TK1 single board computer, running Ubuntu 18.10.

## Hardware Components
* NVIDIA Jetson TK1 single board computer development kit
* USB camera
* Laptop computer running Forza Motorsport and xbox 360 controller emulator


## Dependencies
The following third party libraries are required to build and run this project:
* OpenCV2


## How to Build 

1. On the Jetson TK1, clone the ECEN5623_final repository into a direcotry named "ECEN5623_final"
2. cd ECEN5623_final/RealTime/
3. make


## How to Run
1. On the external laptop launch Forza Motorsport, and the xbox 360 controller emulator
2. Select the race track, and begin the race.
3. On the Jetson TK1, "cd ECEN5623_final/RealTime/"
4. On the Jetson TK1, Launch the controller application by running the command "./drive". The application will begin capturing images of the laptop screen and sending commands to the controller emulator to drive the car. 




 
