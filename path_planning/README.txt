This directory contains the various MATLAB files used for path
planning/trajectory planning on the robot.

The path planning code is organized as follows:
Level 1: Throw and catch a ball with one hand, and one dimension (up/down)
Level 2: Throw-Throw-Catch-Catch motion with two balls, two hands, two
dimensions

To Run the path planner (Plot a cycle):
For Level 1: Open up the plot_cycle.m file in matlab, and uncomment the
             path_planner field (line 6) to equal 'path_planning_level1_v10'
             and the path_gen_hand1 (line 8) to equal 'arm1path_level1_v10'.
             path_gen_hand2 should be commented out (line  9 + 10).
             Comment out path_gen_fun_hand2 (line 15)
             Update the t_final and the resolution appropriately.  
             Open up path_planning_level1_v10 and change any robot,
             workspace, or ball trajectory parameters. When you are
             satisfied, hit run on the plot_cycle script. 

For Level 2: Open up the plot_cycle.m file in MATLAB, and uncomment the
             path_planner field (line 7)to equal 'path_planning_level2_v02'
             and the path_gen_hand1 (line 9) to equal 'arm1path_level1_v01',
             and the path_gen_hand2 (line 10) to equal 'arm2path_level2_v01'
             uncomment path_gen_fun_hand2 (line 15)
             Update the t_final and the resolution appropriately.  
             Open up path_planning_level2_v02 and change any robot,
             workspace, or ball trajectory parameters. When you are
             satisfied, hit run on the plot_cycle script. 

To Export as CSV:
Open up csv_path_export.m
Update the values in path_planner (line 15), and path_gen (line 16) and, for
level 2, path_gen in line 110 (comment this if you are doing level 1), as
appropriate for the level, as explained for the plot_cycle script. Modify
the parameters in path_planning_level??_v?? as appropriate.
Uncomment/Comment whichever csvwrite() commands you want to get the desired
csv files. CSV files can be written for linear position, velocity,
acceleration, and angular position, velocity and acceleration for the X and
Y motors in the gantry with pulley radius r_pulley (line 17). 


The robot simulator is a rudimentary physics engine that simulates a robot 
hand moving along a trajectory, in order to throw and catch a ball. Right 
now, the hand-ball interactions don't quite work well, and it is only set up
to work with level 1, but it is useful for observing the hand motion of a 
given robot trajectory. To try it out, run the robot_simulator file. Again,
modify parameters in the path_planning files. 
