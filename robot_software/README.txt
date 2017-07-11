This is the main software to control the Juggling Robot.

To compile it, you must have the PIGPIO library installed on a Raspberry Pi.
When this is done, you can just run the makefile. 

All major setup parameters are in main.cpp

In general, there is a dc_motor class that is created for each motor, and a
rot_encoder class that is created for the rotary encoder attached to the
motor. Each dc_motor is assigned its proper encoder, and all necessary
control parameters are passed to the proper objects. 

A homing sequence is then performed to determine how many encoder counts
constitute the robots rectangular workspace (along each motor axis).

The user is then prompted a menu of options to choose from in the robot
control.


An in-depth explanation of the code in main.cpp is set up as follows:

A pin sample time and PWM frequency are chosen (see PIGPIO Docs)

encoder_velocity_points - Since the encoders are incremental (1024 PPR),
getting a velocity involves differentiation. This parameter determines how
many past velocity points to differentiate over to get the velocity at each control loop
iteration. 

udp_port_number: an agreed upon port number for the comptuer running the CV
software to send messages over UDP to the Raspberry Pi. 

MOTOR SETUP:
Control constants (feedforward, open loop, Kp and Kd) are set. This
transforms values in the input text files to PWM duty cycles (0-255)

Input velocity and distance files are chosen for each motor. 

Proper pins on the Raspberry Pi are set for the encoders, motor drivers,
limit switches, homing speed (up/down PWM), workspace width, limit switch
width and a correction factor for directions (motors are mounted
symmetrically, so counter clockwise could be up or down depending on the
specific Y motor). 

PIGPIO Library setup:
See PIGPIO Docs. 

Encoder Setup:
Encoder objects are created for each encoder, and fed the proper data.

UDP Comm Setup:
A UDP communication wrapper object is created, and assigned a port number.
It is activated, and will start listening for UDP messages from the computer
vision computer to feed to the robot if it is running a mode where it is
listening for UDP signals. Only the X axis motors can be controlled over
UDP. The UDP messages should be strings in the form:
"R0.125 L0.02" to command the right X to go to 0.125 and the left X to go to 0.02
To stop the UDP transmission, the sender can send: "XXXXX"

MOTOR OBJECT SETUP:
Here, the actual motor objects are created. Note that the objects are fed
references to the appropriate encoder. 

MOTOR HOMING:
Here, the motors are homed. Note that because of space limitation on the Pi,
some upper switches are connected together and some lower switches are connected
together. If any axis fails to home, it will be added to a list, and one
more attempt to home it will be carried out after the other axes are
attempted. 

UI CONTROL FLOW:
Here, the user is prompted to input which control mode he or she desires.
Note that the furthest we developed was the throw-catch sequence (one hand
to the other), with potential CV control over UDP. 

1) Point control - User inputs a point, robot moves to that point. Each axis
is prompted sequentially. 
2) Left Hand One Axis Throw-Catch: Left hand performs a throw-catch sequence
in the Y
3) Right Hand One Axis Throw-catch: Right hand performs a throw-catch
sequence in the Y
4) Double One Hand One Axis Throw-Catch: Both hand simultaneously perform a
throw-catch.
5) Open Loop Throw-Catch - One hand throws the ball to the other hand. This
is called open loop since the CV is not used to correct the second hand. All
positions and velocities are controlled however. 
6) Closed Loop (CV aided) Throw-Catch: One hand throws to the other. Once
the left hand throws, the right hand X is put under the control of the CV
via UDP messaging. 
7) Kinect following the ball: This is a neat demo where the CV is given
control over the hand. A user can move the ball in the robot workspace and
the user can watch the robot track the ball in real time.

For modes that require multiple axes to move at the same time, a struct is
made the contains a chosen time point, a delay time, and the motor axis for
each motor. Since all motors have the same time point and delay time, all
motors will run in a synchronized fashion after the delay has passed. This
uses the multithreading functions in the PIGPIO library. 

To Terminate, the UDP connection is killed, and all encoders are
deactivated. This is important because it kills processes that constantly
listen to the encoder pins. 

