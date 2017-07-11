/* main.cpp

   David Verdi
   Created 4/16/2017
   Modified 4/16/2017

   This is the main file for controlling the juggling robot.

   Version 1.0: This enters an open loop control sequence for one motor
   with one encoder, using one thread, and one pre-planned angular velocity
   trajectory. 
*/

#include <stdint.h>
#include <limits> 
#include <iostream>
#include <vector>
#include <sstream>
#include <pigpio.h>
#include "dc_motor.hpp"
#include "motor_sync.hpp"
#include "udp_connection.hpp"
#include "main.hpp"

// Sample at a rate of 4 microseconds, PWM of 10 kHz
#define PIN_SAMPLE_TIME 4 
#define PWM_FREQUENCY 10000


using namespace std;

int main(int argc, char *argv[])
{
  //--------------------------------
  //---------GENERAL SETUP----------
  //--------------------------------
  int encoder_velocity_points = 5;
  string udp_port_number = "5005";

  //--------------------------------
  //------LEFT Y MOTOR SETUP--------
  //--------------------------------
  // Set control constants: 
  double LY_open_loop_pwm_constant = 103.59;
  double LY_velocity_feedforward_constant = 60;
  double LY_Kp = 0;
  double LY_Kd = 200;//for perfect catch from before

  // Set velocity file and distance files
  string LY_VelocityFile = "y_throw_velocity_higher_throw_50cm.txt"; // "y_circle.txt";
  string LY_DistanceFile = "y_throw_position_higher_throw_50cm.txt"; // "y_circle.txt";

  int LY_pwm_pin = 25;
  int LY_dir_pin = 8;
  int LY_upper_limit_switch_pin = 6;
  int LY_lower_limit_switch_pin = 13;
  int LY_encoder_A_pin = 14;
  int LY_encoder_B_pin = 15;
  int LY_encoder_Z_pin = 18;
  int LY_up_pwm = 65;
  int LY_down_pwm = 45;
  double LY_workspace_width = 0.45;
  double LY_limit_width = 0.543;
  int LY_direction_factor = 1;
  motor_axis LY_axis = LY;



  //--------------------------------
  //------LEFT X MOTOR SETUP--------
  //--------------------------------
  double LX_open_loop_pwm_constant = 103.59;
  double LX_velocity_feedforward_constant = 200; //103;
  double LX_Kp = 0;
  double LX_Kd = 200;
  double LX_kinect_constant = 300;
 
  // Set velocity file and distance files
  string LX_VelocityFile = "x_throw_velocity_calculated.txt";
  string LX_DistanceFile = "x_throw_velocity_calculated.txt";

  int LX_pwm_pin = 12;
  int LX_dir_pin = 7;
  int LX_upper_limit_switch_pin = 6;
  int LX_lower_limit_switch_pin = 13;
  int LX_encoder_A_pin = 16;
  int LX_encoder_B_pin = 20;
  int LX_encoder_Z_pin = 21;
  int LX_up_pwm = 65;
  int LX_down_pwm = 65;
  double LX_workspace_width = 0.2;
  double LX_limit_width = 0.296;
  int LX_direction_factor = 1;
  motor_axis LX_axis = LX;

  //--------------------------------
  //------RIGHT Y MOTOR SETUP-------
  //--------------------------------
  // Set control constants: 
  double RY_open_loop_pwm_constant = 103.59;
  double RY_velocity_feedforward_constant = 70;
  double RY_Kp = 0;
  double RY_Kd = 200;

  // Set velocity file and distance files
  string RY_VelocityFile = "y_throw_velocity_higher_throw_50cm.txt"; //"level_one_desired_velocity_40cm.txt"; // "y_circle.txt";
  string RY_DistanceFile = "y_throw_position_higher_throw_50cm.txt";

  int RY_pwm_pin = 26;
  int RY_dir_pin = 19;
  int RY_upper_limit_switch_pin = 4;
  int RY_lower_limit_switch_pin = 5;
  int RY_encoder_A_pin = 27; //changed encoder pins to reverse directions after the motor was reflected
  int RY_encoder_B_pin = 17;
  int RY_encoder_Z_pin = 22;
  int RY_up_pwm = 65;
  int RY_down_pwm = 45;
  double RY_workspace_width = 0.45;
  double RY_limit_width = 0.549;
  int RY_direction_factor = 1;
  motor_axis RY_axis = RY;

  //--------------------------------
  //------RIGHT X MOTOR SETUP-------
  //--------------------------------
  double RX_open_loop_pwm_constant = 103.59;
  double RX_velocity_feedforward_constant = 100; //103;
  double RX_Kp = 0;
  double RX_Kd = 200;
  double RX_kinect_constant = 1000;

  // Set velocity file and distance files
  string RX_VelocityFile = "constant_0_500ms.txt"; // "x_half_circle.txt";
  string RX_DistanceFile = "constant_0_500ms.txt"; // "x_half_circle.txt";

  int RX_pwm_pin = 24;
  int RX_dir_pin = 23;
  int RX_upper_limit_switch_pin = 4;
  int RX_lower_limit_switch_pin = 5;
  int RX_encoder_A_pin = 10;
  int RX_encoder_B_pin = 9;
  int RX_encoder_Z_pin = 11;
  int RX_up_pwm = 53;
  int RX_down_pwm = 53;
  double RX_workspace_width = 0.2;
  double RX_limit_width = 0.297;
  int RX_direction_factor = 1;
  motor_axis RX_axis = RX;


  //--------------------------------
  //-----PIGPIO LIBRARY SETUP-------
  //--------------------------------
  // Set up pin sampling rate. 
  // A sampling rate of 4 uS allows PWM signals of up to 10 kHz.
  // Third argument is depricated and ignored. 
  gpioCfgClock(PIN_SAMPLE_TIME, 1, 1);

  // Initialize the PIGPIO library. 
  if (gpioInitialise() < 0) {
    cout << "pigpio library failed to initialize. Exiting Now." << endl; 
    return 1;
  }
  
  // Immediatley set all PWM outputs as low to prevent any motors running.
  gpioSetMode(LY_pwm_pin, PI_OUTPUT);
  gpioWrite(LY_pwm_pin, 0);


  //--------------------------------
  //---------ENCODER SETUP----------
  //--------------------------------
  rot_encoder LY_encoder(LY_encoder_A_pin, LY_encoder_B_pin, LY_encoder_Z_pin, encoder_velocity_points);
  rot_encoder LX_encoder(LX_encoder_A_pin, LX_encoder_B_pin, LX_encoder_Z_pin, encoder_velocity_points);
  rot_encoder RY_encoder(RY_encoder_A_pin, RY_encoder_B_pin, RY_encoder_Z_pin, encoder_velocity_points);
  rot_encoder RX_encoder(RX_encoder_A_pin, RX_encoder_B_pin, RX_encoder_Z_pin, encoder_velocity_points);


  //--------------------------------
  //---------UDP COMM SETUP---------
  //--------------------------------
  udp_connection udp_comm(udp_port_number);
  int udpflag = udp_comm.start_listening();  


  //--------------------------------
  //-------MOTOR OBJECT SETUP-------
  //--------------------------------
  // Prototype
  // dc_motor(int DirPin, int PwmPin, int PwmFreq, int ULimitSwitch, int LLimitSwitch, rot_encoder* enc)
  dc_motor LY_motor(LY_axis, LY_dir_pin, LY_pwm_pin, PWM_FREQUENCY, LY_upper_limit_switch_pin, LY_lower_limit_switch_pin, &LY_encoder);
  LY_motor.set_distance_file(LY_DistanceFile);
  LY_motor.set_velocity_file(LY_VelocityFile);
  LY_motor.set_direction_factor(LY_direction_factor);
  LY_motor.set_constants(LY_open_loop_pwm_constant, LY_velocity_feedforward_constant, LY_Kp, LY_Kd);
  LY_motor.set_homing_parameters(LY_limit_width, LY_workspace_width, LY_up_pwm, LY_down_pwm);
  

  dc_motor LX_motor(LX_axis, LX_dir_pin, LX_pwm_pin, PWM_FREQUENCY, LX_upper_limit_switch_pin, LX_lower_limit_switch_pin, &LX_encoder);
  LX_motor.set_distance_file(LX_DistanceFile);
  LX_motor.set_velocity_file(LX_VelocityFile);
  LX_motor.set_direction_factor(LX_direction_factor);
  LX_motor.set_constants(LX_open_loop_pwm_constant, LX_velocity_feedforward_constant, LX_Kp, LX_Kd);
  LX_motor.set_homing_parameters(LX_limit_width, LX_workspace_width, LX_up_pwm, LX_down_pwm);
  LX_motor.set_kinect_constant(LX_kinect_constant);
  LX_motor.add_comm(&udp_comm);


  dc_motor RY_motor(RY_axis, RY_dir_pin, RY_pwm_pin, PWM_FREQUENCY, RY_upper_limit_switch_pin, RY_lower_limit_switch_pin, &RY_encoder);
  RY_motor.set_distance_file(RY_DistanceFile);
  RY_motor.set_velocity_file(RY_VelocityFile);
  RY_motor.set_direction_factor(RY_direction_factor);
  RY_motor.set_constants(RY_open_loop_pwm_constant, RY_velocity_feedforward_constant, RY_Kp, RY_Kd);
  RY_motor.set_homing_parameters(RY_limit_width, RY_workspace_width, RY_up_pwm, RY_down_pwm);
  

  dc_motor RX_motor(RX_axis, RX_dir_pin, RX_pwm_pin, PWM_FREQUENCY, RX_upper_limit_switch_pin, RX_lower_limit_switch_pin, &RX_encoder);
  RX_motor.set_distance_file(RX_DistanceFile);
  RX_motor.set_velocity_file(RX_VelocityFile);
  RX_motor.set_direction_factor(RX_direction_factor);
  RX_motor.set_constants(RX_open_loop_pwm_constant, RX_velocity_feedforward_constant, RX_Kp, RX_Kd);
  RX_motor.set_homing_parameters(RX_limit_width, RX_workspace_width, RX_up_pwm, RX_down_pwm);
  RX_motor.set_kinect_constant(RX_kinect_constant);
  RX_motor.add_comm(&udp_comm);
  
  //--------------------------------
  //----------MOTOR HOMING----------
  //--------------------------------
  // Home each motor. If any motor fails to home, if both limit switches are
  // pressed, the program exits.
  // Motor homes return 0 if all is well, 1 from unsucessful home that can be
  // recovered from, and 2 for unhomable conditions (more than one limit switch
  // hit on a frame.)

  vector<dc_motor*> fail_list;
  // Home LY_motor
  cout << "Homing LY Motor." << cout;
  switch (LY_motor.home()) 
  {
    case 0: break;
    case 1: fail_list.push_back(&LY_motor);
            cout << "Motor failed to home, trying again in a bit..." << endl;
            break;
    case 2: cout << "Motor failed to home." << endl;
            LY_encoder.deactivate();
            LX_encoder.deactivate();
            gpioTerminate();
            return 0;
  }

  // Home LX_motor. 
  cout << "Homing LX Motor." << cout;
  switch (LX_motor.home()) 
  {
    case 0: break;
    case 1: fail_list.push_back(&LX_motor);
            cout << "Motor failed to home, trying again in a bit..." << endl;
            break;
    case 2: cout << "Motor failed to home." << endl;
            LY_encoder.deactivate();
            LX_encoder.deactivate();
            gpioTerminate();
            return 0;
  }

  // Home RX_motor. 
  cout << "Homing RX Motor." << cout;
  switch (RX_motor.home()) 
  {
    case 0: break;
    case 1: fail_list.push_back(&RX_motor);
            cout << "Motor failed to home, trying again in a bit..." << endl;
            break;
    case 2: cout << "Motor failed to home." << endl;
            RY_encoder.deactivate();
            RX_encoder.deactivate();
            gpioTerminate();
            return 0;
  }

  // Home RY_motor

  cout << "Homing RY Motor." << cout;
  switch (RY_motor.home()) 
  {
    case 0: break;
    case 1: fail_list.push_back(&RY_motor);
            cout << "Motor failed to home, trying again in a bit..." << endl;
            break;
    case 2: cout << "Motor failed to home." << endl;
            RY_encoder.deactivate();
            RX_encoder.deactivate();
            gpioTerminate();
            return 0;
  }
  

  
  // Home motors that failed to home originally.
  for(int i = 0; i < ((int) (fail_list.size())); i++)
  {
    // If it fails again, forget about it. 
    if((fail_list[i])->home())
    {
      LY_encoder.deactivate();
      LX_encoder.deactivate();
      RY_encoder.deactivate();
      RX_encoder.deactivate();
      gpioTerminate();
      return 0;
    }
  }


  //--------------------------------
  //---------UI CONTROL FLOW--------
  //--------------------------------
  cout << "Homing Sequence is completed." << endl;
  string instring;
  int invalue;

  while(true)
  {
    cout << "Please enter your desired operating mode" << endl;
    cout << "Your options are: " << endl;
    cout << "1. Point Control" << endl;
    cout << "2. Left Hand One Axis Throw-Catch" << endl;
    cout << "3. Right Hand One Axis Throw-Catch" << endl;
    cout << "4. Double One Hand One Axis Throw-Catch" << endl;
    cout << "5. Open Loop Throw-Catch (Left to Right)" << endl;
    cout << "6. Closed-Loop (CV aided) Throw-Catch (Left to Right)" << endl;
    cout << "7. Kinect following the ball" << endl;
    cout << "8. EXIT" << endl;
    cout << ">> ";

    getline(cin,instring);
    stringstream(instring) >> invalue;
    if (invalue == 8)
      break;

    switch (invalue)
    {
      case 1: main_point_control(&LY_motor, &LX_motor, &RY_motor, &RX_motor);
              break;
      case 2: main_l1htc(&LY_motor, &LX_motor, &RY_motor, &RX_motor); // One hand throw-catch
              break;
      case 3: main_r1htc(&LY_motor, &LX_motor, &RY_motor, &RX_motor); // One hand throw-catch
              break;
      case 4: main_double_1htc(&LY_motor, &LX_motor, &RY_motor, &RX_motor); //Double one hand throw catch
              break;
      case 5: main_ol_tc(&LY_motor, &LX_motor, &RY_motor, &RX_motor);
              break;        
      case 6: main_cl_tc(&LY_motor, &LX_motor, &RY_motor, &RX_motor);
              break;
      case 7: main_kinect(&LY_motor, &LX_motor, &RY_motor, &RX_motor);
              break;
    }
  }

  //--------------------------------
  //--------Program Termination-----
  //--------------------------------
  udp_comm.kill_connection();
  LY_encoder.deactivate();
  LX_encoder.deactivate();
  RY_encoder.deactivate();
  RX_encoder.deactivate();
  gpioTerminate();
  return 0;
}



void main_point_control(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor)
{
  LX_motor->point_control();
  LY_motor->point_control();
  RX_motor->point_control();
  RY_motor->point_control();
  return;
}

void main_l1htc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor)
{
  //--------------------------------
  //------ONE HAND THROW CATCH------
  //--------------------------------
  uint32_t tick = gpioTick();
  int delay_seconds = 1;
  cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;
  LY_motor->run_pdff_path(tick, delay_seconds);
}
void main_r1htc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor)
{
  //--------------------------------
  //------ONE HAND THROW CATCH------
  //--------------------------------
  uint32_t tick = gpioTick();
  int delay_seconds = 1;
  cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;
  RY_motor->run_pdff_path(tick, delay_seconds);
}

void main_double_1htc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor)
{
  uint32_t tick = gpioTick();
  int delay_seconds = 5;
  int timeout = 12;
  cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;


  // Create motor sync_struct objects. Since we can only pass one agument into
  // the thread creation function, we package all of our necessary data into a 
  // sync_struct so that the thread has everything it needs to run!
  motor_sync_struct LY_sync_struct = make_sync_struct(LY_motor, tick, delay_seconds);
  motor_sync_struct RY_sync_struct = make_sync_struct(RY_motor, tick, delay_seconds);

  pthread_t *pt_LX, *pt_LY, *pt_RX, *pt_RY;

  // Create a thread for each motor. 
  // Each thread calls the sync_pdff function, which runs the closed loop
  // PD-FeedForward loop on the motor named in the sync_struct. 
  pt_LY = gpioStartThread(sync_pdff, &LY_sync_struct);
  pt_RY = gpioStartThread(sync_pdff, &RY_sync_struct);

  // Poll the active motors to ensure the motors are done running.
  // When all of them report that they are done, kill the threads to be safe.
  // Poll every second.

  while(!(LY_motor->all_done()) || !(RY_motor->all_done()))
  {
    gpioSleep(PI_TIME_RELATIVE, 3, 0);
  }


  LX_motor->all_done_flag = false;
  LY_motor->all_done_flag = false;
  RX_motor->all_done_flag = false;
  RY_motor->all_done_flag = false;

  gpioStopThread(pt_LY);
  gpioStopThread(pt_RY);
}

void main_ol_tc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor)
{
  //--------------------------------
  //----------OPEN LOOP-----------
  //----------THROW-CATCH-----------
  //--------------------------------
  
  uint32_t tick = gpioTick();
  int delay_seconds = 5;
  cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;


  // Create motor sync_struct objects. Since we can only pass one agument into
  // the thread creation function, we package all of our necessary data into a 
  // sync_struct so that the thread has everything it needs to run!
  motor_sync_struct LY_sync_struct = make_sync_struct(LY_motor, tick, delay_seconds);
  motor_sync_struct LX_sync_struct = make_sync_struct(LX_motor, tick, delay_seconds);
  motor_sync_struct RY_sync_struct = make_sync_struct(RY_motor, tick, delay_seconds);
  motor_sync_struct RX_sync_struct = make_sync_struct(RX_motor, tick, delay_seconds);

  pthread_t *pt_LX, *pt_LY, *pt_RX, *pt_RY;

  // Create a thread for each motor. 
  // Each thread calls the sync_pdff function, which runs the closed loop
  // PD-FeedForward loop on the motor named in the sync_struct. 
  pt_LY = gpioStartThread(sync_pdff, &LY_sync_struct);
  pt_LX = gpioStartThread(sync_pdff, &LX_sync_struct);
  pt_RY = gpioStartThread(sync_pdff, &RY_sync_struct);
  pt_RX = gpioStartThread(sync_pdff, &RX_sync_struct);

  // Poll the active motors to ensure the motors are done running.
  // When all of them report that they are done, kill the threads to be safe.
  // Poll every second.

  while(!(LX_motor->all_done()) || !(LY_motor->all_done()) || !(RX_motor->all_done()) || !(RY_motor->all_done()))
  {
    gpioSleep(PI_TIME_RELATIVE, 3, 0);
  }

  LX_motor->all_done_flag = false;
  LY_motor->all_done_flag = false;
  RX_motor->all_done_flag = false;
  RY_motor->all_done_flag = false;

  gpioStopThread(pt_LX);
  gpioStopThread(pt_LY);
  gpioStopThread(pt_RX);
  gpioStopThread(pt_RY);
}
                   
void main_cl_tc(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor)
{
  //--------------------------------
  //----------CLOSED LOOP-----------
  //----------THROW-CATCH-----------
  //--------------------------------
  
  uint32_t tick = gpioTick();
  int delay_seconds = 5;
  int timeout = 12;
  cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;


  // Create motor sync_struct objects. Since we can only pass one agument into
  // the thread creation function, we package all of our necessary data into a 
  // sync_struct so that the thread has everything it needs to run!
  motor_sync_struct LY_sync_struct = make_sync_struct(LY_motor, tick, delay_seconds);
  motor_sync_struct LX_sync_struct = make_sync_struct(LX_motor, tick, delay_seconds);
  motor_sync_struct RY_sync_struct = make_sync_struct(RY_motor, tick, delay_seconds);
  motor_sync_struct RX_sync_struct = make_sync_struct(RX_motor, tick, timeout);

  pthread_t *pt_LX, *pt_LY, *pt_RX, *pt_RY;

  // Create a thread for each motor. 
  // Each thread calls the sync_pdff function, which runs the closed loop
  // PD-FeedForward loop on the motor named in the sync_struct. 
  pt_LY = gpioStartThread(sync_pdff, &LY_sync_struct);
  pt_LX = gpioStartThread(sync_pdff, &LX_sync_struct);
  pt_RY = gpioStartThread(sync_pdff, &RY_sync_struct);
  pt_RX = gpioStartThread(sync_pdff, &RX_sync_struct);

  // Poll the active motors to ensure the motors are done running.
  // When all of them report that they are done, kill the threads to be safe.
  // Poll every second.

  while(!(LX_motor->all_done()) || !(LY_motor->all_done()) || !(RX_motor->all_done()) || !(RY_motor->all_done()))
  {
    gpioSleep(PI_TIME_RELATIVE, 3, 0);
  }

  LX_motor->all_done_flag = false;
  LY_motor->all_done_flag = false;
  RX_motor->all_done_flag = false;
  RY_motor->all_done_flag = false;

  gpioStopThread(pt_LX);
  gpioStopThread(pt_LY);
  gpioStopThread(pt_RX);
  gpioStopThread(pt_RY);
}

void main_kinect(dc_motor* LY_motor, dc_motor* LX_motor, dc_motor* RY_motor, dc_motor* RX_motor)
{
  //--------------------------------
  //--------KINECT TRACKING---------
  //--------------------------------
  cout << "Starting Kinect Tracking!" << endl;
  int timeout = 20; //seconds
  RX_motor->follow_kinect(timeout);
  return;
}




// OLD CODE SNIPPETS
  //--------------------------------
  //--------OPEN LOOP CONTROL-------
  //--------------------------------
  //uint32_t tick = gpioTick();
  //int delay_seconds = 1;
  //cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;
  //LY_motor.run_ol_path(tick, delay_seconds);


  //--------------------------------
  //--------CLOSED LOOP CONTROL-----
  //--------------------------------
  //uint32_t tick = gpioTick();
  //int delay_seconds = 1;
  //cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;
  //LY_motor.run_pdff_path(tick, delay_seconds);

  //uint32_t tick = gpioTick();
  //int delay_seconds =1; 
  //cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;
  //RX_motor.run_pdff_path(tick, delay_seconds);





  //--------------------------------
  //----------SYNCHRONIZED---------
  //----------THROW-CATCH----------
  //-------------------------------
  // Left frame will throw, right frame will catch.
/*
  uint32_t tick = gpioTick();
  int delay_seconds = 5;
  int udp_timeout = 15; //Seconds
  cout << "Motors will activate in: " << delay_seconds << " seconds." << endl;


  // Create motor sync_struct objects. Since we can only pass one agument into
  // the thread creation function, we package all of our necessary data into a 
  // sync_struct so that the thread has everything it needs to run!
  motor_sync_struct LY_sync_struct = make_sync_struct(&LY_motor, tick, delay_seconds);
  motor_sync_struct LX_sync_struct = make_sync_struct(&LX_motor, tick, delay_seconds);
  motor_sync_struct RY_sync_struct = make_sync_struct(&RY_motor, tick, delay_seconds);
  motor_sync_struct RX_sync_struct = make_sync_struct(&RX_motor, tick, udp_timeout);

  pthread_t *pt_LX, *pt_LY, *pt_RX, *pt_RY;

  // Create a thread for each motor. 
  // Each thread calls either the the sync_pdff function, which runs the closed loop
  // PD-FeedForward loop on the motor named in the sync_struct, or the sync_kinect,
  // which makes it immediatley wait for orders from the kinect. 
  pt_LY = gpioStartThread(sync_pdff, &LY_sync_struct);
  pt_LX = gpioStartThread(sync_pdff, &LX_sync_struct);
  pt_RY = gpioStartThread(sync_pdff, &RY_sync_struct);
  pt_RX = gpioStartThread(sync_kinect, &RX_sync_struct);

  // Poll the active motors to ensure the motors are done running.
  // When all of them report that they are done, kill the threads to be safe.
  // Poll every second.

  while(!(LX_motor.all_done()) || !(LY_motor.all_done()) || !(RX_motor.all_done()) || !(RY_motor.all_done()))
  {
    gpioSleep(PI_TIME_RELATIVE, 3, 0);
  }

  gpioStopThread(pt_LX);
  gpioStopThread(pt_LY);
  gpioStopThread(pt_RX);
  gpioStopThread(pt_RY);
  */
