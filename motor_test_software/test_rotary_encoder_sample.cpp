/* test_rotary_encoder_sample.cpp
   
   David Verdi
   Created 4/12/2017
   Modified 4/12/2017

   This is an adaptation of the sample rotary encoder
   test file from the pigpio library. 
   I set it with an increased sample rate.
   */


#include <iostream>
#include <pigpio.h>
#include "rotary_encoder_sample.hpp"

// Sample at a rate of 2 microseconds
#define PIN_SAMPLE_TIME 4

//These are the GPIO pins to wire the encoder to
#define ENCODER_PIN_A 27
#define ENCODER_PIN_B 17

void callback(int way)
{
   static int pos = 0;

   pos += way;

   std::cout << "pos=" << pos << std::endl;
}

int main(int argc, char *argv[])
{
  // Set up pin sampling rate. A sampling rate of 2 uS uses 16% of CPU and
  // allows PWM signals of up to 20 kHz. This is the maximum that our motor
  // controllers can handle. Must be called before gpioInitialize
  // second argument of 1 uses PCM to get clock
  // Third argument is depricated and ignored. 
   gpioCfgClock(PIN_SAMPLE_TIME, 1, 1); 
   
   if (gpioInitialise() < 0) return 1;

   re_decoder dec(ENCODER_PIN_A, ENCODER_PIN_B, callback);
   //re_decoder::re_decoder(int gpioA, int gpioB, re_decoderCB_t callback)
   //Calls the constructor re_decoder

   //sleep(3000);

   //Sleep for 30 seconds....
   gpioSleep(PI_TIME_RELATIVE, 20,0);

   dec.re_cancel();

   gpioTerminate();
}


