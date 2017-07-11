/* dc_motor.cpp

   David Verdi
   Created 4/16/2017
   Modified 4/28/2017

   This is the cpp file holding the class function definitions for the dc_motor 
   class. 

   This is version 1.0 so only support for start flags, done flags, one velocity
   time vector and one go flag. 
*/

#include <iostream>
#include <pigpio.h>
#include <cmath>
#include <fstream>
#include <stdlib.h> 
#include <sstream>
#include "dc_motor.hpp"

using namespace std;

// Default constructor definition:
dc_motor::dc_motor()
{
	path_start_flag = false;
	path_done_flag = false;
	kinect_start_flag = false;
	kinect_done_flag = false;
	all_done_flag = false;
	home_flag = false;
	dir_factor = 1;
	pwm_constant = 0;
	current_pwm = 0;
	dir_pin = 0;
	pwm_pin = 0;
	u_limit_switch = 0;
	l_limit_switch = 0;
	limit_latch = false;
	move_precision = 2;
	workspace_width = 0;
	workspace_width_count = 0;
	min_up_pwm = 0;
	min_down_pwm = 0;
	count_per_meter = 0;
	velocity_ff_constant = 0.0;
	proportional_constant = 0.0;
	derivative_constant = 0.0;
	limit_width = 0;
	kinect_constant = 0;

	// Pointers
	encoder = NULL;
	udp_comm = NULL;
}

// Constructor:
dc_motor::dc_motor(motor_axis MotorAxis, int DirPin, int PwmPin, int PwmFreq, int ULimitSwitch, int LLimitSwitch, rot_encoder* enc)
{
	// Input variable settings
	axis = MotorAxis;
	dir_pin = DirPin;
	pwm_pin = PwmPin;
	u_limit_switch = ULimitSwitch;
	l_limit_switch = LLimitSwitch;
	encoder = enc;

	// Input function settings
	gpioSetMode(dir_pin, PI_OUTPUT);
	gpioSetPWMfrequency(pwm_pin, PwmFreq);
	gpioSetMode(ULimitSwitch, PI_INPUT);
	gpioSetMode(LLimitSwitch, PI_INPUT);

    // Important Default Settings
    dir_factor = 1;
    move_precision = 2;

	// Standard Variable Settings
	path_start_flag = false;
	path_done_flag = false;
	kinect_start_flag = false;
	kinect_done_flag = false;
	all_done_flag = false;
	home_flag = false;
	pwm_constant = 0;
	current_pwm = 0;
	limit_latch = false;
	workspace_width = 0;
	workspace_width_count = 0;
	min_up_pwm = 0;
	min_down_pwm = 0;
	count_per_meter = 0;
	velocity_ff_constant = 0.0;
	proportional_constant = 0.0;
	derivative_constant = 0.0;
	d_pulley =  0.0652015;
	limit_width = 0;
	kinect_constant = 0;
	udp_comm = NULL;
}

// Destructor:
dc_motor::~dc_motor()
{
	this->deactivate_limit_latching();
}

// Set the precomputed distance file
void dc_motor::set_distance_file(string distanceFile)
{
	// Initialize variables to hold data
	string line;
	double value;

    // Open file
    ifstream myfile(distanceFile.c_str());

	if ( myfile.is_open() ) {
		while( getline(myfile, line) ) {

			// Convert it to a floating point number
			value = atof(line.c_str());

			// Push it to the back of the vector
			distance_path.push_back(value);
		}
	}

	else
	{
		cout << "Velocity file failed to open." << endl;
	}

	// Close the file
	myfile.close();
}

// Set the precomputed velocity file
void dc_motor::set_velocity_file(string velocityFile)
{
	// Initialize variables to hold data
	string line;
	double value;

    // Open file
    ifstream myfile(velocityFile.c_str());

	if ( myfile.is_open() ) {
		while( getline(myfile, line) ) {

			// Convert it to a floating point number
			value = atof(line.c_str());

			// Push it to the back of the vector
			velocity_path.push_back(value);
		}
	}
	else
	{
		cout << "Velocity file failed to open." << endl;
	}

	// Close the file
	myfile.close();
}

// Run open-loop velocity path. 
void dc_motor::run_ol_path(uint32_t initialization_tick, int delay_seconds)
{
	// Check if velocity path is set:
	if (velocity_path.empty())
	{
		cout << "Velocity vector is empty. Aborting." << endl;
		return;
	}

	// Check if it has been homed yet
	if (!(home_flag))
	{
		cout << "Home axis first. Aborting." << endl;
		return;
	}

	// start_tick is the system tick that all the motors are given. In order to 
	// ensure that all motors start at the same time, we wait for delay_seconds
	// microseconds in order to allow for the overhead involved in starting the
	// four threads.

	// Convert to microseconds for the tick.
	uint32_t delay_second_cast = (uint32_t) delay_seconds;
	uint32_t start_tick = initialization_tick + delay_second_cast*1000000;
	uint32_t max_time_millis = velocity_path.size() - 1;
	uint32_t current_time_millis;

    // Activate the limit latching!
    this->activate_limit_latching();
	
	while(gpioTick() < start_tick)
		; // Do Nothing

        
        printf("Starting Motor! \n");


	// For each time step we need to get the current time in milliseconds since
	// the motor started, grab the correct angular velocity from the path 
	// planning vector, convert it to PWM range (0-255), and apply it to the
	// motor.

	// Fancy changes for the future:
	// - Scale it between min_pwm instead of 0
	// - Do an offset so by the time it actually applies the gpio command it
	//   knows what to do.

	path_start_flag = true;
	current_time_millis = (gpioTick() - start_tick)/1000;

	while (!(limit_latch) && (current_time_millis < max_time_millis) ) {

		// Grab current angular speed from vector
		double angular_speed = dir_factor * velocity_path[current_time_millis];
		// Note that for DIR Pins, 0 is up and 1 is down. 
		if (angular_speed < 0) {
            gpioWrite(dir_pin, 1);
        }
        else {
        	gpioWrite(dir_pin, 0);
        }
		// Convert angular speed to a duty cycle from 0-255
		int duty_cycle = abs(round(pwm_constant * angular_speed));

		// Clip duty_cycle at 255 using a ternary operator. 
		duty_cycle = ((duty_cycle > 255) ? 255 : duty_cycle);

		// Change output PWM if duty_cycle is different from what it is
		// already outputing
		if (current_pwm != duty_cycle) {
			gpioPWM(pwm_pin, duty_cycle);
                        current_pwm = duty_cycle;
                        cout << "Time is: "<< current_time_millis << "  Activating Motor with duty cycle: " << duty_cycle << endl;
		}

		if ((encoder->getCount() > workspace_width_count) || (encoder->getCount() < (-50)))
		{
			cout << "Motor went past workspace. Aborting." << endl;
			this->stop();
			break;
		}

		double encoder_velocity = encoder->getCPS();
		cout << "Current Encoder Velocity (Counts per second) is: " << encoder_velocity << endl;
		current_time_millis = (gpioTick() - start_tick)/1000;
	}
    
    // Make sure it's off!
    cout << "Turning off motor" << endl;
    this->stop();
    this->deactivate_limit_latching();
	path_done_flag = true;
	all_done_flag = true;
}

void dc_motor::run_pdff_path(uint32_t initialization_tick, int delay_seconds)
{
	// Check if velocity path is set:
	if (velocity_path.empty())
	{
		printf("Velocity vector is empty. Aborting.\n");
		return;
	}

	// Check if distance path is set:
	if (distance_path.empty())
	{
		printf("Distance vector is empty. Aborting.\n");
		return;
	}
		// Check if it has been homed yet
	if (!(home_flag))
	{
		printf("Home axis first. Aborting.\n");
		return;
	}

	// start_tick is the system tick that all the motors are given. In order to 
	// ensure that all motors start at the same time, we wait for delay_seconds
	// microseconds in order to allow for the overhead involved in starting the
	// four threads.

	// Convert to microseconds for the tick.
	uint32_t delay_second_cast = (uint32_t) delay_seconds;
	uint32_t start_tick = initialization_tick + delay_second_cast*1000000;
	uint32_t max_time_millis = velocity_path.size() - 1;
	uint32_t current_time_millis;

    // Activate the limit latching!
    this->activate_limit_latching();
	
	while(gpioTick() < start_tick)
		; // Do Nothing

    printf("Starting Motor!\n");

	// For each time step we need to get the current time in milliseconds since
	// the motor started, grab the correct angular velocity from the path 
	// planning vector, convert it to PWM range (0-255), Apply the control law,
	// clip it at 255, then activate the motor. 

	// Fancy changes for the future:
	// - Scale it between min_pwm instead of 0
	// - Do an offset so by the time it actually applies the gpio command it
	//   knows what to do.

	// DATA OUTPUT Vectors
	vector<uint32_t> tdata(max_time_millis + 1);
	vector<double> vdata(max_time_millis + 1);
	vector<double> ddata(max_time_millis + 1);


	path_start_flag = true;
	current_time_millis = (gpioTick() - start_tick)/1000;
	int j = 0;
	uint32_t prev_time_millis = 0;

	while (!(limit_latch) && (current_time_millis < max_time_millis) ) 
	{
		// Get desired velocity (m/s) and desired distance (m) from file.
		// Do all computation in meters/s and meters. 
		double v_d = velocity_path[current_time_millis]; // meters/sec
		double d_d = distance_path[current_time_millis]; // meters

		// Get current position and velocity from encoders (m/s, m)
		// Convert counts per second into meters/sec
		double v = (encoder->getCPS())/count_per_meter;
		double d = (encoder->getCount())/count_per_meter;
		//cout << "Current Linear Velocity is: " << v << endl;
		//cout << "Current Position (m) is: " << d << endl;
		if (prev_time_millis != current_time_millis)
		{
		   vdata[j] = v;
		   ddata[j] = d;
		   tdata[j] = current_time_millis;
		   j++;
		}
		prev_time_millis = current_time_millis;

		double control_law = velocity_ff_constant*v_d + proportional_constant*(d_d - d) + derivative_constant*(v_d - v);

		control_law *= dir_factor;

		if (control_law < 0) {
            gpioWrite(dir_pin, 1);
        }
        else {
        	gpioWrite(dir_pin, 0);
        }

        int duty_cycle = abs(round(control_law));

        // Clip duty_cycle at 255 using a ternary operator. 
		duty_cycle = ((duty_cycle > 255) ? 255 : duty_cycle);

		// Change output PWM if duty_cycle is different from what it is
		// already outputing
		if (current_pwm != duty_cycle) {
			gpioPWM(pwm_pin, duty_cycle);
            current_pwm = duty_cycle;
            //cout << "Time is: "<< current_time_millis << "  Activating Motor with duty cycle: " << duty_cycle << endl;
		}

		if ((encoder->getCount() > (150+workspace_width_count)) || (encoder->getCount() < (-50)))
		{
			printf("Encoder Count: %d", encoder->getCount());
			printf("Motor went past workspace. Aborting \n");
			this->stop();
			break;
		}
		current_time_millis = (gpioTick() - start_tick)/1000;
	}




	// Make sure it's off!
    printf("Turning off motor\n");
    this->stop();
    this->deactivate_limit_latching();
	path_done_flag = true;
	

	string motor_name = enum2string(axis);
	string toutfile = "time_data_" + motor_name + ".txt";
	string voutifle = "velocity_data_" + motor_name + ".txt";
	string doutfile = "distance_data_" + motor_name + ".txt";

	// Open files
    ofstream tfile(toutfile.c_str());
    ofstream vfile(voutifle.c_str());
    ofstream dfile(doutfile.c_str());

    if ((tfile.is_open()) && (vfile.is_open()) && (dfile.is_open()))
    {
    	printf("Writing Files. \n");
    	for (int i = 0; i < ((int) tdata.size()); i++)
    	{
    		tfile << tdata[i] << "\n";
    		dfile << ddata[i] << "\n";
    		vfile << vdata[i] << "\n";
    	}
    }

	tfile.close();
	vfile.close();
	dfile.close();
	all_done_flag = true;
}

void dc_motor::run_pdff_cv_path(uint32_t InitializationTick, int DelaySeconds)
//DEFAULT 15 second timeout.
{
	int timeout = 15; //Seconds. THIS IS HARDCODED. CHANGE IN FUTURE.
	if (!(home_flag))
	{
		printf("Home axis first. Aborting. \n");
		return;
	}

	if (kinect_constant == 0)
	{
		printf("Error: Set Kinect constant first. Aborting \n");
		return;
	}

	if (!udp_comm)
	{
		printf("Error: Set up UDP connection first. Aborting.\n");
		return;
	}

    //THIS BEGINS THE PATH FOLLOWING PORTION. 

	// Check if velocity path is set:
	if (velocity_path.empty())
	{
		printf("Velocity vector is empty. Aborting.\n");
		return;
	}

	// Check if distance path is set:
	if (distance_path.empty())
	{
		printf("Distance vector is empty. Aborting.\n");
		return;
	}
		// Check if it has been homed yet
	if (!(home_flag))
	{
		printf("Home axis first. Aborting.\n");
		return;
	}

	// start_tick is the system tick that all the motors are given. In order to 
	// ensure that all motors start at the same time, we wait for delay_seconds
	// microseconds in order to allow for the overhead involved in starting the
	// four threads.

	// Convert to microseconds for the tick.
	uint32_t delay_second_cast = (uint32_t) DelaySeconds;
	uint32_t start_tick = InitializationTick + delay_second_cast*1000000;
	uint32_t max_time_millis = velocity_path.size() - 1;
	uint32_t current_time_millis;

    // Activate the limit latching!
    this->activate_limit_latching();
	
	while(gpioTick() < start_tick)
		; // Do Nothing

    printf("Starting Motor!\n");

	// For each time step we need to get the current time in milliseconds since
	// the motor started, grab the correct angular velocity from the path 
	// planning vector, convert it to PWM range (0-255), Apply the control law,
	// clip it at 255, then activate the motor. 

	// Fancy changes for the future:
	// - Scale it between min_pwm instead of 0
	// - Do an offset so by the time it actually applies the gpio command it
	//   knows what to do.

	// DATA OUTPUT Vectors
	vector<uint32_t> tdata(max_time_millis + 1);
	vector<double> vdata(max_time_millis + 1);
	vector<double> ddata(max_time_millis + 1);


	path_start_flag = true;
	current_time_millis = (gpioTick() - start_tick)/1000;
	int j = 0;
	uint32_t prev_time_millis = 0;

	while (!(limit_latch) && (current_time_millis < max_time_millis) ) 
	{
		// Get desired velocity (m/s) and desired distance (m) from file.
		// Do all computation in meters/s and meters. 
		double v_d = velocity_path[current_time_millis]; // meters/sec
		double d_d = distance_path[current_time_millis]; // meters

		// Get current position and velocity from encoders (m/s, m)
		// Convert counts per second into meters/sec
		double v = (encoder->getCPS())/count_per_meter;
		double d = (encoder->getCount())/count_per_meter;
		//cout << "Current Linear Velocity is: " << v << endl;
		//cout << "Current Position (m) is: " << d << endl;
		if (prev_time_millis != current_time_millis)
		{
		   vdata[j] = v;
		   ddata[j] = d;
		   tdata[j] = current_time_millis;
		   j++;
		}
		prev_time_millis = current_time_millis;

		double control_law = velocity_ff_constant*v_d + proportional_constant*(d_d - d) + derivative_constant*(v_d - v);

		control_law *= dir_factor;

		if (control_law < 0) {
            gpioWrite(dir_pin, 1);
        }
        else {
        	gpioWrite(dir_pin, 0);
        }

        int duty_cycle = abs(round(control_law));

        // Clip duty_cycle at 255 using a ternary operator. 
		duty_cycle = ((duty_cycle > 255) ? 255 : duty_cycle);

		// Change output PWM if duty_cycle is different from what it is
		// already outputing
		if (current_pwm != duty_cycle) {
			gpioPWM(pwm_pin, duty_cycle);
            current_pwm = duty_cycle;
            //cout << "Time is: "<< current_time_millis << "  Activating Motor with duty cycle: " << duty_cycle << endl;
		}

		if ((encoder->getCount() > (150+workspace_width_count)) || (encoder->getCount() < (-50)))
		{
			printf("Encoder Count: %d", encoder->getCount());
			printf("Motor went past workspace. Aborting \n");
			this->stop();
			break;
		}
		current_time_millis = (gpioTick() - start_tick)/1000;
	}

	// Make sure it's off!
    printf("Turning off motor\n");
    this->stop();
    this->deactivate_limit_latching();
	path_done_flag = true;

	


    //*****************************************
	//THIS BEGINS CV PORTION....
	//*****************************************
	uint32_t timeout_tick = gpioTick() + ((uint32_t) timeout)*1000000;

	while(!(udp_comm->track_flag) && (gpioTick() < timeout_tick))
	{
		; //Do Nothing...
	}

	if((gpioTick() > timeout_tick))
	{
		printf("Timed Out \n");
		return;
	} 

	while((udp_comm->track_flag) && (gpioTick() < timeout_tick))
	{
		double d = (encoder->getCount())/count_per_meter;
		double d_d;

		switch (axis)
		{
			case RX: d_d = (udp_comm->get_RX())*count_per_meter;
			         break;
			case LX: d_d = (udp_comm->get_LX())*count_per_meter;
			         break;
			case LY: printf("Can't follow kinect on Y Motor. Aborting. \n");
					 return;
			case RY: printf("Can't follow kinect on Y Motor. Aborting. \n");
					 return;
		}

		if ((d_d > (50+workspace_width_count)) || (d_d < (-50)))
		{
			printf("CV command is: %f This is past the workspace. Aborting.", d_d);
			this->stop();
			break;
		}

		//CONTROL LAW
		double control_law = kinect_constant*(d_d - d) - 50;

		control_law *= dir_factor;

		if (control_law < 0) {
            gpioWrite(dir_pin, 1);
        }
        else {
        	gpioWrite(dir_pin, 0);
        }

        int duty_cycle = abs(round(control_law));

        // Clip duty_cycle at 255 using a ternary operator. 
		duty_cycle = ((duty_cycle > 255) ? 255 : duty_cycle);

		// Change output PWM if duty_cycle is different from what it is
		// already outputing
		if (current_pwm != duty_cycle) {
			gpioPWM(pwm_pin, duty_cycle);
            current_pwm = duty_cycle;
		}

		if ((encoder->getCount() > (50+workspace_width_count)) || (encoder->getCount() < (-50)))
		{
		    cout << "Encoder Count: " << encoder->getCount() << endl;
			cout << "Motor went past workspace. Skipping.\n" << endl;
			this->stop();
			continue;
		}
	}

	//FILE PRINTING. PUT AT END, AFTER CV PORTION. 
	string motor_name = enum2string(axis);
	string toutfile = "time_data_" + motor_name + ".txt";
	string voutifle = "velocity_data_" + motor_name + ".txt";
	string doutfile = "distance_data_" + motor_name + ".txt";

	// Open files
    ofstream tfile(toutfile.c_str());
    ofstream vfile(voutifle.c_str());
    ofstream dfile(doutfile.c_str());

    if ((tfile.is_open()) && (vfile.is_open()) && (dfile.is_open()))
    {
    	printf("Writing Files. \n");
    	for (int i = 0; i < ((int) tdata.size()); i++)
    	{
    		tfile << tdata[i] << "\n";
    		dfile << ddata[i] << "\n";
    		vfile << vdata[i] << "\n";
    	}
    }
	tfile.close();
	vfile.close();
	dfile.close();

	all_done_flag = true;
	return;
}

void dc_motor::follow_kinect(int timeout)
{
	if (!(home_flag))
	{
		printf("Home axis first. Aborting. \n");
		return;
	}

	if (kinect_constant == 0)
	{
		printf("Error: Set Kinect constant first. Aborting \n");
		return;
	}

	if (!udp_comm)
	{
		printf("Error: Set up UDP connection first. Aborting.\n");
		return;
	}

	uint32_t timeout_tick = gpioTick() + ((uint32_t) timeout)*1000000;

	while(!(udp_comm->track_flag) && (gpioTick() < timeout_tick))
	{
		; //Do Nothing...
	}

	if((gpioTick() > timeout_tick))
	{
		printf("Timed Out \n");
		return;
	} 
	cout << "Getting data from kinect..." << endl;

	// Activate the limit latching!
    this->activate_limit_latching();

	while((udp_comm->track_flag) && (gpioTick() < timeout_tick))
	{
		double d = (encoder->getCount())/count_per_meter;
		double d_d;

		switch (axis)
		{
			case RX: d_d = (udp_comm->get_RX());
			         break;
			case LX: d_d = (udp_comm->get_LX());
			         break;
			case LY: printf("Can't follow kinect on Y Motor. Aborting. \n");
					 return;
			case RY: printf("Can't follow kinect on Y Motor. Aborting. \n");
					 return;
		}

		//Convert d_d from coordinates from the limit switch origin to
		//coordinates relative to the workspace origin.
		d_d = (d_d - (limit_width - workspace_width)/2);

		// Check if kinect ordering the hand past the workspace. Saturate it at
		// one centimeter before end of workspace.
		if (d_d > workspace_width)
		{
			//d_d = workspace_width - 0.01;
			continue;
		}

		if (d_d < 0)
		{
			//d_d = 0.01;
			continue;
		}

		printf("d: %f\n",d);
		printf("d_d: %f\n", d_d);

		double control_law = kinect_constant*(d_d - d) + (50*((d_d-d)/abs(d_d-d)));
		cout << "Control law: " << control_law << endl;

		control_law *= dir_factor;

		if (control_law < 0) {
            gpioWrite(dir_pin, 1);
        }
        else {
        	gpioWrite(dir_pin, 0);
        }

        int duty_cycle = abs(round(control_law));

        // Clip duty_cycle at 255 using a ternary operator. 
		duty_cycle = ((duty_cycle > 255) ? 255 : duty_cycle);

		// Change output PWM if duty_cycle is different from what it is
		// already outputing
		if (current_pwm != duty_cycle) {
			gpioPWM(pwm_pin, duty_cycle);
            current_pwm = duty_cycle;
		}
	}
	this->deactivate_limit_latching();
	all_done_flag = true;
}

void dc_motor::point_control()
{
	// Check if it has been homed yet
	if (!(home_flag))
	{
		cout << "Home axis first. Aborting." << endl;
		return;
	}

	string instring;
	double invalue;

	// Activate the limit latching!
    this->activate_limit_latching();
    string axis_string = enum2string(axis);

	while(1)
	{

		cout << "Axis: " << axis_string << " Enter your desired position!" << endl;
		cout << ">> ";

		getline(cin,instring);
		stringstream(instring) >> invalue;
		if (invalue < 0)
			break;

		if ((invalue >= workspace_width))
		{
			cout << "\n" << "Entered value is out of bounds!" << endl;
			continue;
		}
		cout << "encoder count is now: " << encoder->getCount() << endl;

		cout << "\nGoing to " << invalue << endl;

		int target_position = round(invalue*count_per_meter);
		cout << "In counts, the target is: " << target_position << " plus/minus " << move_precision << endl;

		int position_reached = 0;

		while((!position_reached) && (!limit_latch))
		{
			if (encoder->getCount() > (target_position + move_precision))
			{
				this->run_speed_no_limit(min_down_pwm, -1);
			}

			if (encoder->getCount() < (target_position - move_precision))
			{
				this->run_speed_no_limit(min_up_pwm, 1);
			}

			if (((encoder->getCount()) > (target_position - move_precision)) && ((encoder->getCount()) < (target_position + move_precision)))
			{
				cout << "Target Position Reached!" << endl;
				position_reached = 1;
				this->stop();
			}
		}
	}

	// Deactivate the limit latching!
    this->deactivate_limit_latching();
}


void dc_motor::go_to_point(double point)
{
	// Check if it has been homed yet
	if (!(home_flag))
	{
		cout << "Home axis first. Aborting." << endl;
		return;
	}

	// Activate the limit latching!
	this->activate_limit_latching();

	if ((point >= workspace_width) || (point < -50))
	{
		cout << "\n" << "Entered value is out of bounds!" << endl;
		return;
	}
	cout << "encoder count is now: " << encoder->getCount() << endl;

	cout << "\nGoing to " << point << endl;

	int target_position = round(point*count_per_meter);
	cout << "In counts, the target is: " << target_position << " plus/minus " << move_precision << endl;

	int position_reached = 0;

	while((!position_reached) && (!limit_latch))
	{
		if (encoder->getCount() > (target_position + move_precision))
		{
			this->run_speed_no_limit(min_down_pwm, -1);
		}

		if (encoder->getCount() < (target_position - move_precision))
		{
			this->run_speed_no_limit(min_up_pwm, 1);
		}

		if (((encoder->getCount()) > (target_position - move_precision)) && ((encoder->getCount()) < (target_position + move_precision)))
		{
			cout << "Target Position Reached!" << endl;
			position_reached = 1;
			this->stop();
		}
	}

	// Deactivate the limit latching!
	this->deactivate_limit_latching();
}


// Runs at a duty cycle. Direction: +1 for up and -1 for down.
// Note that for DIR Pins, 0 is up and 1 is down. 
void dc_motor::run_speed_no_limit(int duty_cycle, int direction)
{
	direction *= dir_factor;
	if (direction < 0) {
            gpioWrite(dir_pin, 1);
        }
        else {
        	gpioWrite(dir_pin, 0);
        }

	//cout << "Activating motor with duty_cycle: " << duty_cycle << endl;
	gpioPWM(pwm_pin, duty_cycle);
	current_pwm = duty_cycle;
}

void dc_motor::stop()
{
	cout << "Stopping Motor" << endl;
	gpioPWM(pwm_pin, 0);
}

void dc_motor::activate_limit_latching()
{
	// Set Alert functions
	gpioSetAlertFuncEx(u_limit_switch, _static_limit_hit, this);
	gpioSetAlertFuncEx(l_limit_switch, _static_limit_hit, this);
}

void dc_motor::_static_limit_hit(int gpio_caller, int level, uint32_t tick, void *userdata)
{
	dc_motor* Self = (dc_motor*) userdata;
	Self->_limit_hit(gpio_caller, level);
}

void dc_motor::_limit_hit(int gpio_caller, int level)
{
	if((level == 0) && !(gpioRead(gpio_caller)))
	{
		this->stop();
		limit_latch = true;
		cout << "Limit Switch Hit!" << endl;
	}
	return;
}

void dc_motor::deactivate_limit_latching()
{
	gpioSetAlertFuncEx(u_limit_switch, 0, this);
	gpioSetAlertFuncEx(l_limit_switch, 0, this);
}

void dc_motor::reset_limit_latches()
{
	limit_latch = false;
}

void dc_motor::set_direction_factor(int factor)
{
	if(abs(factor) == 1)
		dir_factor = factor;
	else
	{
		cout << "Invalid direction factor. Must be +1 or -1" << endl;
		return;
	}
}

int dc_motor::home()
{
	// Returns 0 if homing all good
	// Returns 1 if homing is not good, but can be fixed if run again after other
	// axis homes. 
	// Returns 2 if homing failed catastrophically

	if (workspace_width > limit_width)
	{
	        cout << "HE Workspace width is : " << workspace_width << endl;
		cout << "limit width is: " << limit_width << endl;
		cout << "ERROR from homing: Limit width smaller than workspace width!" << endl;
		return(2);
	}

	if(!gpioRead(l_limit_switch) && !gpioRead(u_limit_switch))
	{
		cout << "Two limit switches hit on a frame. Please manually move one." << endl;
		cout << "Aborting now." << endl;
		return(2);
	}

	encoder->resetCount();

	// Clear lower limit, only while upper limit is clear, and only while it is going
	// less than 300 counts. 
	while(!gpioRead(l_limit_switch) && gpioRead(u_limit_switch) && ((encoder->getCount()) < 300))
	{
		// Lower limit switch being touched. Move up!
		this->run_speed_no_limit(min_up_pwm, 1);
	}
	this->stop();

	// At this point, if the upper limit switch is hit/encoder reads 280+, and the lower limit switch is still hit, 
	// we know that the other motor is also hitting a switch. We back down to zero and flag a fixable problem.
	if(!gpioRead(l_limit_switch) && (!gpioRead(u_limit_switch) || ((encoder->getCount()) > 280)))
	{
		//Back down to the place where we started.
		while((encoder->getCount()) > 2)
		{
			this->run_speed_no_limit(min_down_pwm, -1);
		}
		this->stop();
		return(1);
	}
	this->stop();

	encoder->resetCount();
	// Clear upper limit, only while lower limit is clear, and only while it is going less than 300 counts. 
	while(!gpioRead(u_limit_switch) && gpioRead(l_limit_switch) && ((encoder->getCount()) > -300))
	{
		// upper limit switch being touched. Move down!
		this->run_speed_no_limit(min_down_pwm, -1);
	}
	this->stop();

	//At this point, if the lower limit swich is hit/encoder reads -280+ AND the upper switch is still hit,
	// the problem lies in the opposite axis. Return fixable fail.
	if(!gpioRead(u_limit_switch) && (!gpioRead(l_limit_switch) || ((encoder->getCount()) < -280)))
	{
		//Back upto the place where we started.
		while((encoder->getCount()) < -2)
		{
			this->run_speed_no_limit(min_up_pwm, 1);
		}
		this->stop();
		return(1);
	}
	this->stop();

	cout << "Step 1: Moving down until bottom limit switch is touched." << endl;
	// Move down until we touch lower limit switch
	while(1)
	{
	    if(!gpioRead(l_limit_switch))
		{
		    gpioSleep(PI_TIME_RELATIVE, 0, 20);
		if(!gpioRead(l_limit_switch))
		{
	        this->stop();
		    break;
		}
		}
		this->run_speed_no_limit(min_down_pwm, -1);
	}
	this->stop();

	// Set encoder count to zero at bottom
    cout << "Hit Bottom. Encoder count set to zero at bottom!" << endl; 
	encoder->resetCount();

	cout << "Step 2: Moving up until top limit switch will be hit..." << endl;
	// Slowly move up until top limit switch is hit:
	while(1)
	{
		if(!gpioRead(u_limit_switch))
		{
		    gpioSleep(PI_TIME_RELATIVE, 0, 20);
			if(!gpioRead(u_limit_switch))
			{
				this->stop();
				break;
			}
		}
		this->run_speed_no_limit(min_up_pwm, 1);
	}
	this->stop();

	double span_count = (double) (encoder->getCount());
	cout << "Hit the top. The counts from switch to switch is: " << span_count << endl;

	count_per_meter = (span_count / limit_width);
	cout << "The count per meter is: " << count_per_meter << endl;

	workspace_width_count = workspace_width * count_per_meter;
	cout << "The counts in the workspace is: " << workspace_width_count << endl;

	double limit_buffer = 0.5 * (span_count - workspace_width_count);
	cout << "The Limit buffer is: " << limit_buffer << endl;

	// Go to zero point, which is current zero + limit_buffer
	cout << "Going to zero position" << endl;
	int target_position = (int) round(limit_buffer);
	cout << "Target is: " << target_position << endl;

	// Clear upper limit. 
	while (!gpioRead(u_limit_switch))
	{
	        cout << "Moving off Top limit switch" << endl;
		// upper limit switch being touched. Move down!
		this->run_speed_no_limit(min_down_pwm, -1);
	}

	this->stop();

	int position_set = 0;

	while((!position_set)) 
	{
	    //cout << "Entering zero loop" << endl;
		if (encoder->getCount() > (target_position + move_precision))
		{
		        
		       // cout << "Encoder Count too high, Moving down: " << encoder->getCount() << endl;
			this->run_speed_no_limit(min_down_pwm, -1);
		}

		if (encoder->getCount() < (target_position - move_precision))
		{
		        
		       // cout << "Encoder Count too low, Moving up" << encoder->getCount() << endl;
			this->run_speed_no_limit(min_up_pwm, 1);
		}

		if (((encoder->getCount()) > (target_position - move_precision)) && ((encoder->getCount()) < (target_position + move_precision)))
			{
			    cout << "Just Right!" << endl;
				position_set = 1;
				this->stop();
			}
	}

	//Pause for half a second
	gpioSleep(PI_TIME_RELATIVE, 0, 500000);

	//Second while loop:
	position_set = 0;
	
	while((!position_set)) 
	{
	    //cout << "Entering zero loop" << endl;
		if (encoder->getCount() > (target_position + move_precision))
		{
		        
		       // cout << "Encoder Count too high, Moving down: " << encoder->getCount() << endl;
			this->run_speed_no_limit(min_down_pwm, -1);
		}

		if (encoder->getCount() < (target_position - move_precision))
		{
		        
		       // cout << "Encoder Count too low, Moving up" << encoder->getCount() << endl;
			this->run_speed_no_limit(min_up_pwm, 1);
		}

		if (((encoder->getCount()) > (target_position - move_precision)) && ((encoder->getCount()) < (target_position + move_precision)))
			{
			    cout << "Just Right!" << endl;
				position_set = 1;
				this->stop();
			}
	}


	cout << "Pre-Reset, the count is now at: " << (encoder->getCount()) << endl;

	encoder->resetCount();
	cout << "Zeroing Done!" << endl;
	home_flag = 1;
	cout << "Motor is now  at: " << (encoder->getCount()) << endl;
	return(0);
}

void dc_motor::set_precision_factor(int factor)
{
	move_precision = factor;
}

void dc_motor::set_constants(double pwmConstant, double velocityFFconstant, double Kp, double Kd)
{
	pwm_constant = pwmConstant;
	velocity_ff_constant = velocityFFconstant;
	proportional_constant = Kp;
	derivative_constant = Kd;
}

bool dc_motor::all_done()
{
	return all_done_flag;
}

void dc_motor::set_homing_parameters(double limitWidth, double workspaceWidth, int upPWM, int downPWM)
{
	min_up_pwm = upPWM;
	min_down_pwm = downPWM;
	workspace_width = workspaceWidth;
	limit_width = limitWidth;

	if (workspace_width > limit_width)
	{
		cout << "limit width is: " << limit_width << endl;
		cout << "workspace width is: " << workspace_width << endl;
		cout << "ERROR: Limit width smaller than workspace width!" << endl;
		return;
	}
	return;
}

void dc_motor::set_kinect_constant(double constnt)
{
	kinect_constant = constnt;
	return;
}

void dc_motor::add_comm(udp_connection* comm)
{
	if (comm)
	{
		udp_comm = comm;
	}
}


string enum2string(motor_axis axis)
{
	string output;
	switch (axis)
	{
		case LY: output = "LY";
		         break;
		case LX: output = "LX";
		         break;
		case RY: output = "RY";
		         break;
		case RX: output = "RX";
				break;
	}
	return(output);
}
