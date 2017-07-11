/* rot_encoder.hpp

   David Verdi
   Created 4/16/2017
   Modified 4/20/2017

   This is the header file holding the function definitions for the rot_encoder
   class. 
   
   This is heavily modeled after the example rotary encoder code given with the
   pigpio library. 
*/

#ifndef __ROT_ENCODER_HPP__
#define __ROT_ENCDOER_HPP__

#include <pthread.h>
#include <deque>


class rot_encoder
{

public:

	// PUBLIC VARIALBES

	// Holds current absolute position in cumulative pulse counts
	volatile int pulse_count;

	// How many samples to save in memory
	int deque_width;

	// Mutex to ensure exclusive acess to the deques
	pthread_mutex_t deque_lock;

    // queue to hold count and timestamp history for velocity calculation
	std::deque<int> count_deque;
	std::deque<uint32_t> time_deque;

	// Encoder input pin numbers
	int a_pin;
	int b_pin;
	int z_pin;

	// Variable to store the current level of the pin
	volatile int a_level;
	volatile int b_level;

	// Variable to hold last GPIO which changed (to debounce)
	volatile int last_pin_change;

	// PUBLIC FUNCTIONS
	
	// Default Constructor
	rot_encoder();

	// Constructor
	rot_encoder(int a_pin, int b_pin, int z_pin, int velocity_points);

	// Destructor
	~rot_encoder();

	// Static function to be called when encoder goes high
    static void _static_pulse(int gpio_caller, int level, uint32_t tick, void *userdata);

	void _pulse(int gpio_caller, int level, uint32_t tick);

	// Pulse function for index pin
	static void z_static_pulse(int gpio_caller, int level, uint32_t tick, void *userdata);

    void z_pulse(int gpio_caller, int level, uint32_t tick);

    // Returns the current speed in counts per second. 
	double getCPS();

    // Release resources taken up by encoder
    void deactivate();

    // Get the current count
    int getCount();

    // Test the encoder picking up 1024 signals per revolution.
    void testEncoder();

    // Sets the count to be zero
    void resetCount();

private:

	// PRIVATE FUNCTIONS
	// Disable default copy constructor, and move constructor
	rot_encoder(const rot_encoder&);
	rot_encoder& operator=(const rot_encoder&);
};

#endif
