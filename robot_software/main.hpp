/* main.cpp

   David Verdi
   Created 5/2/2017
   Modified 5/2/2017

   This is the header file for main. It contians all of the subfunctions. 
*/

#ifndef __MAIN_HPP__
#define __MAIN_HPP__

void main_point_control(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor);

void main_r1htc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor); // One hand throw-catch

void main_l1htc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor); // One hand throw-catch

void main_double_1htc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor); //Double one hand throw catch

void main_ol_tc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor);
                   
void main_cl_tc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor);

void main_kinect(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor);

#endif
