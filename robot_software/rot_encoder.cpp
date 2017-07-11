/* rot_encoder.cpp

   David Verdi
   Created 4/16/2017
   Modified 4/28/2017

   This is the cpp file holding the function definitions for the rot_encoder
   class. 
   
   This is heavily modeled after the example rotary encoder code given with the
   pigpio library. 
*/

#include <iostream>
#include <pigpio.h>
#include "rot_encoder.hpp"

using namespace std;

// Default constructor:
rot_encoder::rot_encoder()
{
	pulse_count = 0;
	deque_width = 0;
	a_pin = 0;
	b_pin = 0;
	z_pin = 0;
	a_level = 0;
	b_level = 0;
	last_pin_change = -1;
}

// Constructor
rot_encoder::rot_encoder(int aPin, int bPin, int zPin, int velocity_points)
{
	pulse_count = 0;
	a_pin = aPin;
	b_pin = bPin;
	z_pin = zPin;
	a_level = 0;
	b_level = 0;
	last_pin_change = -1;
	deque_width = velocity_points;

	// Initialize the deque mutex
	pthread_mutex_init(&deque_lock, NULL);

	// Initialize deques for time and position
	deque<int> count_init (deque_width, 0);
	count_deque = count_init;
	deque<uint32_t> time_init (deque_width, 0);
	time_deque = time_init;

	// Set encoder pins as inputs
	gpioSetMode(a_pin, PI_INPUT);
	gpioSetMode(b_pin, PI_INPUT);
	gpioSetMode(z_pin, PI_INPUT);

	// Set pull-up resistors
    gpioSetPullUpDown(a_pin, PI_PUD_UP);
    gpioSetPullUpDown(b_pin, PI_PUD_UP);
    gpioSetPullUpDown(z_pin, PI_PUD_UP);

    // Monitor pin level changes and launch _static_pulse
    // with arguments GPIO pin, new level, tick, and pointer to user data
    gpioSetAlertFuncEx(a_pin, _static_pulse, this);
    gpioSetAlertFuncEx(b_pin, _static_pulse, this);
}

// Destructor
rot_encoder::~rot_encoder()
{
	pthread_mutex_destroy(&deque_lock);
	this->deactivate();
}

// Static pulse function
void rot_encoder::_static_pulse(int gpio_caller, int level, uint32_t tick, void *userdata)
{
	rot_encoder *Self = (rot_encoder *) userdata;
	// Send to non-static pulse function
	Self->_pulse(gpio_caller, level, tick);
}

// Pulse Function
void rot_encoder::_pulse(int gpio_caller, int level, uint32_t tick)
{
	if (gpio_caller == a_pin)
		a_level = level;
	else
		b_level = level;

	// Debounce
	if (gpio_caller != last_pin_change)
	{
		last_pin_change = gpio_caller;

		// Positive Code - Arm is going up
		if ((gpio_caller == a_pin) && (level == 1 ) && (b_level))
		{
			// Positive Code Here
			++pulse_count;
			//cout << "Encoder Count: " << pulse_count << endl;

			// Get Mutex, Add in new point at back, remove front point.
			{
			        pthread_mutex_lock(&deque_lock);
				count_deque.push_back((int) pulse_count);
				time_deque.push_back(tick);
				count_deque.pop_front();
				time_deque.pop_front();
			        pthread_mutex_unlock(&deque_lock);
				return;
			}
			
		}

		// Arm is going down
		else if ((gpio_caller == b_pin) && (level == 1 ) && (a_level))
		{
			// Negative Code Here
			--pulse_count;
			//cout << "Encoder Count: " << pulse_count << endl;

			// Get Mutex, Add in new point at back, remove front point.
			{
			    
			        pthread_mutex_lock(&deque_lock);
				count_deque.push_back((int) pulse_count);
				time_deque.push_back(tick);
				count_deque.pop_front();
				time_deque.pop_front();
			        pthread_mutex_unlock(&deque_lock);
				return;
			}
		}
	}
}

// Index Static pulse function
void rot_encoder::z_static_pulse(int gpio_caller, int level, uint32_t tick, void *userdata)
{
	rot_encoder *Self = (rot_encoder *) userdata;
	// Send to non-static pulse function
	Self->z_pulse(gpio_caller, level, tick);
}

// Pulse Function
void rot_encoder::z_pulse(int gpio_caller, int level, uint32_t tick)
{
	if (level == 1)
	{
		int saved_pulse_count = pulse_count;
		pulse_count = 0;
		cout << "Index pin hit! Pulse count since last hit was: " << saved_pulse_count << endl;
	}
}

// Return current speed
double rot_encoder::getCPS()
{
	// Get Mutex, make local copies of deque. 
	// Make local copies of the deques, so that it doesnt
	// get overwritten while the function is processing. 
	deque<int> local_count;
	deque<uint32_t> local_time;

	{
	    pthread_mutex_lock(&deque_lock);
		local_count = count_deque;
		local_time = time_deque;
	    pthread_mutex_unlock(&deque_lock);
	}

	if ((((int) local_time.size()) != deque_width) || (((int) local_count.size()) != deque_width))
	{
		cout << "Corrupted Deque. Returning zero velocity." << endl;
		return(0);
	} 

    // Update with most recent time point...
//	local_count.push_back((int) pulse_count);
//	local_time.push_back(gpioTick());
//	local_count.pop_front();
//	local_time.pop_front();

	//cout << "Current Time Deque: ";
	//for(int i = 0; i < deque_width; i++)
	//{
//		cout << local_time[i] << " ";
//	}

//	cout << "Current Count Deque: ";
//	for(int i = 0; i < deque_width; i++)
//	{
//		cout << local_count[i] << " ";
//	}

	// Least Squares:
	double xsum = 0;
	double ysum = 0;
	double xxsum = 0;
	double xysum = 0;
	for(int i = 0; i < deque_width; i++)
	{
		xsum += ( (double) local_time[i]);
		ysum += ((double) local_count[i]);
		xxsum += (( (double) (local_time[i]))  * ( (double) local_time[i]));
		xysum += (((double) local_time[i]) * ((double) local_count[i]));
	}

	double denom = ((deque_width * xxsum) - (xsum * xsum));

	if (denom == 0)
	{
		cout << "Singular Matrix. Returning Zero velocity" << endl;
		return(0);
	}
	
//	cout << "xsum: " << xsum << endl;
//	cout << "ysum: " << ysum << endl;
//	cout << "xxsum: " << xxsum << endl;
//	cout << "xysum: " << xysum << endl;

	double slope = ((deque_width * xysum) - (xsum * ysum)) / denom;

    // Multiply slope by conversion factor to get counts per second. Return counts per second. 
	return(slope * 1000000);

	
	// DO LEAST SQUARES HERE....
	// For now I will subtract fifth from first item in the deque
	//double delta_count = local_count.back() - local_count.front();

	// Convert delta_count to revolutions
	//delta_count /= 1024;

	//double delta_time = local_time.back() - local_time.front();
	
	//if (delta_time == 0)
	//	return 0;

	// Convert delta_time to minutes...
	//delta_time /= 60000000;

	//return (delta_count / delta_time);
}

// Release resources taken up by enocder

void rot_encoder::deactivate()
{
	// Remove alert functions from 
	gpioSetAlertFuncEx(a_pin, NULL, this);
    gpioSetAlertFuncEx(b_pin, NULL, this);
}

int rot_encoder::getCount()
{
	return((int) pulse_count);
}

void rot_encoder::testEncoder()
{
	cout << "Activating encoder test..." << endl;
	gpioSetAlertFuncEx(a_pin, z_static_pulse, this);
	gpioSleep(PI_TIME_RELATIVE, 20, 0);
	cout << "Encoder test concluded." << endl;
	gpioSetAlertFuncEx(a_pin, 0, this);
}

void rot_encoder::resetCount()
{
	pulse_count = 0;
}


