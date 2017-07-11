# senior_design_juggling_robot
This is the public repository for the source code used in the Juggling Robot senior design project at Columbia University.
Copyright (c) 2017, David Verdi, Martín L. Perez, Jamar K. Liburd, Jacob M. Greenberg

This repository contains the source code for a robot that is capable of throwing a ball from one hand, and catching it in the other. 
The robot was run off of a Raspbery Pi, with all control software written in C++, using the PIGPIO library. 

The path planning directory contains various MATLAB scripts and functions that were used for path planning and trajectory planning, along with a rudimentary physics engine used to visualize and simulates the movement of the robot's hand and the ball.

The motor_test_software directory contains varous test scripts for running encoders and motors with the PIGPIO library. 

The robot_software directory contains all the final code used for robot operation.

We hope you will find this repository to be a useful starting point in your own robotics projects. 
