/* Threading Example.cpp
   David Verdi
   Created 4/26/2017
   Edited 4/26/2017

   This is a test script to do threading with the pigpio
   library.
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

void *start_LX(void *synch)
{
	int LX_pwm = 12;
	int LX_speed = 80;
	uint32_t initialization_tick = *(uint32_t*) synch;
	uint32_t delay_seconds = 2;
	uint32_t start_tick = initialization_tick + delay_seconds*1000000;

	while(gpioTick() < start_tick)
		; // Do Nothing

    cout << "Starting Motor LX!" << endl;

    int status_flag = gpioPWM(LX_pwm, LX_speed);
    if (status_flag)
		cout << "Error on gpioPWM function LX!" << endl;
    cout << "PWM LX just went on! \n";

    // Pause 0.5 seconds. 
    gpioSleep(PI_TIME_RELATIVE, 1,0);

    status_flag = gpioPWM(LX_pwm, 0);
    if (status_flag)
		cout << "Error on gpioPWM function on LX!" << endl;
    cout << "PWM LX just went off! \n";
    return NULL;
}

void *start_LY(void *synch)
{
	int LY_pwm = 25;
	int LY_speed = 70;
	uint32_t initialization_tick = *(uint32_t*) synch;
	uint32_t delay_seconds = 2;
	uint32_t start_tick = initialization_tick + delay_seconds*1000000;

	while(gpioTick() < start_tick)
		; // Do Nothing

    cout << "Starting Motor LY!" << endl;

    int status_flag = gpioPWM(LY_pwm, LY_speed);
    if (status_flag)
		cout << "Error on gpioPWM function on LY!" << endl;
    cout << "PWM LY just went on! \n";

    // Pause 0.5 seconds. 
    gpioSleep(PI_TIME_RELATIVE, 1,0);

    status_flag = gpioPWM(LY_pwm, 0);
    if (status_flag)
		cout << "Error on gpioPWM function on LY!" << endl;
    cout << "PWM LY just went off! \n";
    return NULL;
}

int main(int argc, char *argv[])
{
	int LX_pwm = 12;
	int LX_dir = 7;

	int LY_pwm = 25;
	int LY_dir = 8;

	int duration = 0.5; //seconds

	gpioCfgClock(PIN_SAMPLE_TIME, 1, 1); 

    // Initialize the PIGPIO library. Return 1 to shell if it sends and error. 
	if (gpioInitialise() < 0)
		return 1;

	// Set the Dir pin as low. This causes Motor Output A to be high and B to be low
    gpioSetMode(LX_dir, PI_OUTPUT);
    gpioWrite(LX_dir, 1);
    gpioSetMode(LY_dir, PI_OUTPUT);
    gpioWrite(LY_dir, 0);
    cout << "direction pin is set to low! \n";

    // Set PWMfrequency
    gpioSetPWMfrequency(LX_pwm, PWM_FREQUENCY);
    gpioSetPWMfrequency(LY_pwm, PWM_FREQUENCY);

    uint32_t tick = gpioTick();

    pthread_t *p1, *p2;
    p1 = gpioStartThread(start_LY, &tick);
    p2 = gpioStartThread(start_LX, &tick);

    gpioSleep(PI_TIME_RELATIVE,10 ,0);

    gpioStopThread(p1);
    gpioStopThread(p2);

    cout << "Ending Main function" << endl;
}
