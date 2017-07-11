/* motor_sync.cpp
 
   David Verdi
   Created 4/26/2017
   Modified 4/26/2017

   This is the c++ source file for motor sync strucutres and the 
   run_motor_sync_pdff function. The run_motor_sync_pdff function takes in
   a pointer to the motor, synchronizing tick, and the delay time.
   It packs them up into a motor_sync_struct, creates a thread, and passes
   back the thread ID. 
*/

#include "motor_sync.hpp"

 /* 
   Struct is as follows:
   struct motor_sync_struct {
      dc_motor *motor;
      uint32_t tick;
      int delay;
   }
*/

motor_sync_struct make_sync_struct(dc_motor *motor, uint32_t tick, int timing)
{
   motor_sync_struct output_struct;
   output_struct.motor = motor;
   output_struct.tick = tick;
   output_struct.timing = timing;
   return output_struct;
}

void *sync_pdff(void *s_struct)
{
   // Cast input pointer to what it really is, a sync_struct*
   motor_sync_struct *struct_ptr = (motor_sync_struct *) s_struct;

   // Unpack struct members
   dc_motor* motor = struct_ptr->motor;
   uint32_t tick = struct_ptr->tick;
   int delay = struct_ptr->timing;

   // Run the motor on a pdff path. 
   motor->run_pdff_path(tick, delay);
   return NULL;
}

void *sync_kinect(void *s_struct)
{
   // Cast input pointer to what it really is, a sync_struct*
   motor_sync_struct *struct_ptr = (motor_sync_struct *) s_struct;

   // Unpack struct members
   dc_motor* motor = struct_ptr->motor;
   int timeout = struct_ptr->timing;

   // Run the motor on a pdff path.
   motor->follow_kinect(timeout); 
   return NULL;
}



