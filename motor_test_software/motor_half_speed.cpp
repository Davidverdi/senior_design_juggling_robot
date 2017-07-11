/* motor_half_speed.cpp

   David Verdi
   Created 4/5/2017
   Modified 4/6/2017

   This is a test script to run the motor at half speed using the PIGPIO.
   library.

   It runs for 10 seconds. 
 */

#include <iostream>
#include <pigpio.h>
using namespace std;

// The duty cycle bit ranges from 0 - 255
#define MIN_DUTY_CYCLE 0
#define MAX_DUTY_CYCLE 255
// Sample at a rate of 4 microseconds, PWM of 10 kHz
#define PIN_SAMPLE_TIME 4
#define PWM_FREQUENCY 10000

int main(int argc, char *argv[])
{
	int freq, status_flag, test_pin, dir_pin;

	// Set the pin that we are going to manipulate (GPIO number from 1-31)
	// Choice GPIO Pin 5, which is physical pin 29;
        // The dir pin is set to be GPIO Pin 6, which is physical pin 31
	test_pin = 25;
  dir_pin = 8;
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

    // Set PWM frequency for the test pin
    freq = gpioSetPWMfrequency(test_pin, PWM_FREQUENCY);
    cout << "Frequency on pin " << test_pin << " is set to: " << freq << "\n";

    // gpioPWM sets a PWM waveform to a pin with the maximum duty cycle
    // It returns 0 if everything went ok. 
        int half_speed = 127;
        cout << "Activating duty cycle of:" << half_speed << "\n";
	status_flag = gpioPWM(test_pin, half_speed);

	// Throw an error if gpioPWM returns anything except 0
	if (status_flag)
		cout << "Error on gpioPWM function!";
        cout << "PWM just went on! \n";

    // Use PIGPIO sleep function to sleep for 10 seconds and 0 microseconds
	gpioSleep(PI_TIME_RELATIVE, 10, 0);

	// Set the PWM to low, to turn off the motor. 
	status_flag = gpioPWM(test_pin, MIN_DUTY_CYCLE);
        cout << "PWM set Low!\n";

        gpioSleep(PI_TIME_RELATIVE, 1,0);

	// Throw an error if gpioPWM returns anything except 0
	if (status_flag)
		cout << "Error on gpioPWM function!";

	// Turn off the PIGPIO library control on the pins
	gpioTerminate();
        
        return 0; 
}
