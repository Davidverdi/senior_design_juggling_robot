/* motor_sync.cpp
 
   David Verdi
   Created 4/26/2017
   Modified 4/26/2017

   Header file for motor_sync.cpp

*/
#ifndef __MOTOR_SYNC_HPP__
#define __MOTOR_SYNC_HPP__

#include <pigpio.h>
#include "dc_motor.hpp"

struct motor_sync_struct {
   dc_motor *motor;
   uint32_t tick;
   int timing;
};

motor_sync_struct make_sync_struct(dc_motor *motor, uint32_t tick, int timing);

void *sync_pdff(void *s_struct);

void *sync_kinect(void *s_struct);

#endif
