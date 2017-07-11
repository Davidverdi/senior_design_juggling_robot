/* dc_motor.hpp
 
   David Verdi
   Created 4/16/2017
   Modified 4/28/2017

   This is the header file holding the function definitions for the dc_motor 
   class. 

   This is version 1.0 so only support for start flags, done flags, one 
   velocity-time vector, and one encoder.

   The variable velocity_path is a pointer to a vector of integers. Each entry
   in this vector refers to the velocity value for each milisecond of the time
   path. 
 */

#ifndef __DC_MOTOR_HPP__
#define __DC_MOTOR_HPP__

#include <vector>
#include <string>
#include "rot_encoder.hpp"
#include "udp_connection.hpp"

enum motor_axis {LY, LX, RY, RX}; 

std::string enum2string(motor_axis axis);

class dc_motor 
{

public:

	// Public Variables:
	motor_axis axis;

	// Flag to tell if motor has started running the velocity vector
	bool path_start_flag;

	// Flag to tell if motor is done
	bool path_done_flag;

	// Flag to tell if it is tracking the kinect
	bool kinect_start_flag;

	// Flag to tell if it is done with the kinect
	bool kinect_done_flag;

	// Flag to signal all done!
	bool all_done_flag; 

	// Flag  to tell if system has been homed yet
	bool home_flag;

	// Direction corrrection factor -  Helps for errors correlating cw and ccw
	// to up-down, left-right. Defaults to one.  
	int dir_factor;

	// Constant to convert from real angular velocity to PWM range (0-255)
	double pwm_constant;

	// Current PWM
	int current_pwm;

	// DIR Pin
	int dir_pin;

	// PWM pin;
	int pwm_pin;

	// Upper limit Switch
	int u_limit_switch;

	// Lower limit switch
	int l_limit_switch;

	// Debounced latch for upper limit switch
	bool limit_latch;

	// Accuracy of count for point-to-point movement
	int move_precision;

	// Workspace width (meters)
	double workspace_width;

	// workspace width (counts)
	int workspace_width_count;

	// Minimum upwards speed
	int min_up_pwm;

	// Minimum downwards speed
	int min_down_pwm;
	
	// Width between limit switches
	double limit_width;

	// Counts per meter (measured)
	double count_per_meter;

	// velocity Feedforward constant 
	// Turns linear meters per second into PWM Duty Cycle
	double velocity_ff_constant;

	// proportional constant for velocity control
	double proportional_constant;

	// Derivative constant for velocity control
	double derivative_constant; 

	// Kinect constant
	double kinect_constant;

	// Pulley Diameter
	double d_pulley;

	// Pointer to an encoder object
	rot_encoder* encoder;

	// Pointer to the velocity path vector
	std::vector<double> velocity_path;

	// Pointer to the distance path vector
	std::vector<double> distance_path;

	// Pointer to a UDP connection object
	udp_connection* udp_comm;

	// Public Functions:
	// Default Constructor
	dc_motor();

	// Constructor
	dc_motor(motor_axis axis, int DirPin, int PwmPin, int PwmFreq, int ULimitSwitch, int LLimitSwitch, rot_encoder* enc);

	// Destructor
	~dc_motor();

	// Set distance path
	void set_distance_file(std::string distanceFile);

	// Set velocity path
	void set_velocity_file(std::string velocityFile);

	// Run Open Loop velocity path
	void run_ol_path(uint32_t InitializationTick, int DelaySeconds);

	// Run PD-Feedforward velocity path
	void run_pdff_path(uint32_t InitializationTick, int DelaySeconds);

	// Runs PD-Feedforward velocity path, then follows kinect.
	// Terminates when ball is caught. 
	void run_pdff_cv_path(uint32_t InitializationTick, int DelaySeconds);

	// Point control -- Goes to user defined point
	void point_control();

	void go_to_point(double point);

	// Follow Kinect variable --  WARNING: ONLY DO THIS ON X MOTORS
	void follow_kinect(int timeout);

    // Runs at a duty cycle. Direction: +1 for up and -1 for down. 
	void run_speed_no_limit(int duty_cycle, int direction);

	// Unconditional motor stop
	void stop();

	// Start polling the pins to latch the limit switches on contact
	void activate_limit_latching();
    
    // stop polling the pins to latch limit switches
	void deactivate_limit_latching();

	static void _static_limit_hit(int gpio_caller, int level, uint32_t tick, void *userdata);

	void _limit_hit(int gpio_caller, int level);

	// reset the limit switches;
	void reset_limit_latches();

	// Set direction factor
	void set_direction_factor(int factor);

    // Homing Sequence. 0 for sucess, 1 for recoverable fail, 2 for unable to home.
	int home();

	// Set the precision factor
	void set_precision_factor(int factor);

	// Set control system constants
	void set_constants(double pwm_constant, double velocityFFconstant, double Kp, double Kd);

	// set the homing parameters
	void set_homing_parameters(double limitWidth, double workspaceWidth, int upPWM, int downPWM);

	// return flag that tells if motor is done running.
	bool all_done();

	// Set proportional constant for kinect tracking
	void set_kinect_constant(double constnt);

	// Add a UDP connection object to let the motor talk to the kinect
	void add_comm(udp_connection* comm);



private:
	// Disable default copy constructor and assignment operator by declaring
	// them private
	dc_motor(const dc_motor&);
	dc_motor& operator=(const dc_motor&);

};

#endif
