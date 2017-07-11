/* motor_switch_test.cpp

   David Verdi
   Created 4/5/2017
   Modified 4/6/2017

   The motor will run at 10/255 speed for 10 seconds.
   During this time, the state of the limit switch will be tested every loop
   iteration to see if it goes low. As soon as it goes low, it will break
   out of the loop and cancel operation. 
 */

#include <iostream>
#include <pigpio.h>
using namespace std;

// The duty cycle argument ranges from 0 - 255
#define RUN_DUTY_CYCLE 150 

// Sample at a rate of 2 microseconds, PWM of 20 kHz
#define PIN_SAMPLE_TIME 2
#define PWM_FREQUENCY 20000

int main(int argc, char *argv[])
{
	int freq, status_flag, pwm_pin, dir_pin, limit_switch_pin;

	// Set the pin that we are going to manipulate (GPIO number from 1-31)
	// Use Pins 2 and 3 (Left MC Y Pins)
	pwm_pin = 2;
    dir_pin = 3;

    // Set up pin sampling rate. A sampling rate of 2 uS uses 16% of CPU and
    // allows PWM signals of up to 20 kHz. This is the maximum that our motor
    // controllers can handle. Must be called before gpioInitialize
    // second argument of 1 uses PCM to get clock
    // Third argument is depricated and ignored. 
    gpioCfgClock(PIN_SAMPLE_TIME, 1, 1); 

    // Initialize the PIGPIO library. Return 1 to shell if it sends and error. 
	if (gpioInitialise() < 0)
		return 1;

    // Set the Dir pin as low. This causes Motor Output A to be high and B to be low
    gpioSetMode(dir_pin, PI_OUTPUT);
    gpioWrite(dir_pin, 0);
    cout << "direction pin is set to low! \n";

    // Set the limit switch pin as an input. Normally it should reat high.
    // It grounds when it hits low. 
    limit_switch_pin = 16;
    gpioSetMode(limit_switch_pin, PI_INPUT);
    cout << "Limit Switch Pin is set as an input! \n";

    // Set PWM frequency for the test pin
    freq = gpioSetPWMfrequency(pwm_pin, PWM_FREQUENCY);
    cout << "Frequency on pin " << pwm_pin << " is set to: " << freq << "\n";
    
    // START THE MOTOR!!
    // gpioPWM sets a PWM waveform to a pin with the maximum duty cycle
    // It returns 0 if everything went ok. 
    cout << "Activating duty cycle of:" << RUN_DUTY_CYCLE << "\n";
	status_flag = gpioPWM(pwm_pin, RUN_DUTY_CYCLE);

	// Throw an error if gpioPWM returns anything except 0
	if (status_flag)
		cout << "Error on gpioPWM function!";
    cout << "PWM just went on! \n";

    // Set up timing scheme with GPIO Tick.
    // A Tick is a 32 bit unsigned integer that represents the time since bootup
    // in microseconds. The Tick overflows every 1 hour and 12 minutes, but all 
    // differences containing one overflow or less are guaranteed to be correct. 
    // i.e. You can time for up to ANY 1 hour and 12 minute time period. 
    uint32_t start_tick, target_tick_diff, current_tick, current_tick_diff;

    // Target is 10 seconds or 1E7 Microseconds. 
    target_tick_diff = 10000000;

    //Begin a Timer
    start_tick = gpioTick();
    current_tick = gpioTick();
    current_tick_diff = current_tick - start_tick;

    // Use a while loop as both a timer and a way to poll the limit switch
    while( (current_tick_diff < target_tick_diff) && (gpioRead(limit_switch_pin)))
    	{
    		current_tick = gpioTick();
            current_tick_diff = current_tick - start_tick;
    	}

	// Set the PWM to low, to turn off the motor. 
	status_flag = gpioPWM(pwm_pin, 0);
    cout << "PWM set Low!\n";

    // Throw an error if gpioPWM returns anything except 0
	if (status_flag)
		cout << "Error on gpioPWM function! \n";

	// Turn off the PIGPIO library control on the pins
	gpioTerminate();
    return 0; 
}
