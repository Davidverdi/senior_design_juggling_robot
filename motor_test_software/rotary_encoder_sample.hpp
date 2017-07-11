/* rotary_encoder_sample.hpp
   
   David Verdi
   Created 4/12/2017
   Modified 4/12/2017

   This is an adaptation of the sample rotary encoder
   test file from the pigpio library. 
   I set it with an increased sample rate.
   */

#ifndef ROTARY_ENCODER_SAMPLE_HPP
#define ROTARY_ENCODER_SAMPLE_HPP

#include <stdint.h>

typedef void (*re_decoderCB_t)(int);

class re_decoder
{
   int mygpioA, mygpioB, levA, levB, lastGpio;

   re_decoderCB_t mycallback;

   void _pulse(int gpio, int level, uint32_t tick);

   /* Need a static callback to link with C. */
   static void _pulseEx(int gpio, int level, uint32_t tick, void *user);


   public:

   re_decoder(int gpioA, int gpioB, re_decoderCB_t callback);
   /*
      This function establishes a rotary encoder on gpioA and gpioB.

      When the encoder is turned the callback function is called.
   */

   void re_cancel(void);
   /*
      This function releases the resources used by the decoder.
   */
};

#endif

