function pp_data = path_planning_level1_v10(simulation_time)
% path_planning_level1_v10.m
% David Verdi
% Created: 2/1/2017
% Updated: 2/1/2017

% WORKS WITH: arm1path_lvel1_v10.m

% IN THIS VERSION: The aceleration is profiled, and the robot arm moves in
% a bang-bang sort of motion, with three on the threow-catch sequence. The
% top of the throw_catch sequence is zero speed. The bottom of the
% catch-throw is set to be zero speed. The output is a pp_data_class

% This function calculates a constrained path for level 1 of success. 
% This path is then used by arm1path_level1 in order to yield the 
% position, velocity, acceleration, and jerk of the arm as a function of
% time. This function was implemented to only have to solve the path
% planning algorithms once, rather than at every time step. 
% The function takes in a simulation time in order to produce the 
% appropriate number of cycles to run. 


% Return Value is of type pp_data_class

% The polynomial for each segment takes zero as its initial time. 

%% Set System Variables (EDIT THESE TO CHANGE PATH)
% Set workspace data
workspace_width = 0.4; %m
workspace_height = 0.4; %m
workspace_gap = 0.1; %m

% Set Ball Trajectory Data
ball_trajectory_height = 0.5; %m
d_throw_y = (3*workspace_height)/4; %m %was workspace_height/2
t_ct = 1; %m

% For this version, the x will take on a constant value from the center
% point to the throw or catch point. The only thing that will change is the
% distance from the edge to the throw and catch point. 
tc_edge_offset = (1/3)*workspace_width;

% Calculate trajectory of ball based on inputs
v_throw_y = sqrt(2 * 9.81 * ball_trajectory_height);
disp(v_throw_y)
t_air = (2 * v_throw_y)/9.81;
v_throw_x = (tc_edge_offset + workspace_gap + (workspace_width - tc_edge_offset))/t_air;
disp(v_throw_x)

%% Initialize output object
pp_data = pp_data_class();

%% Compute workspace data:
workspace1_max_height = workspace_height; %m
workspace1_min_height = 0; %m
workspace1_min_x = workspace_gap/2; %m
workspace1_max_x = workspace_gap/2 + workspace_width; %m

workspace2_max_height = workspace_height; %m
workspace2_min_height = 0; %m
workspace2_min_x = -workspace_gap/2 - workspace_width; %m
workspace2_max_x = -workspace_gap/2;  %m

workspace1_bounding_box = [workspace1_min_x workspace1_max_x workspace1_min_height workspace1_max_height];
workspace2_bounding_box = [workspace2_min_x workspace2_max_x workspace2_min_height workspace2_max_height];
pp_data.set_workspace1(workspace1_bounding_box)
pp_data.set_workspace2(workspace2_bounding_box)

% Set Home positions:
hand1_home_x = workspace1_max_x - 0.5*workspace_width;
hand1_home_y = workspace1_min_height;
hand2_home_x = workspace2_min_x + 0.5*workspace_width;
hand2_home_y = workspace2_min_height;

%% X Path Planning
% X does not move anywhere, so it is the same through the entire
% simulation. Therefore, it is in a permanent rampup period. 
t_ramp_x = simulation_time;
d_poly = hand1_home_x;
v_poly = 0;
a_poly = 0;
j_poly = 0;
pp_data.add_rampup(1, 'x', t_ramp_x, d_poly, v_poly, a_poly, j_poly);

%% Ramp-up Path Planning- Hand1 - Y
% The ramp-up contains one phase: An acceleration of a_ramp.
% a(0) = a_ramp   a(t_ramp) = a_ramp
% v(0) = 0        v(t_ramp) = v_throw
% d(0) = 0        d(t_ramp) = d_throw

% v_f^2 = v_0^2 + 2*a*d
% a = (v_f^2)/(2*d)
a_ramp = (v_throw_y^2)/(2*d_throw_y);

% v_f = v_0 + a*t
% t = v_f/a
t_ramp = v_throw_y/a_ramp;

a_poly = a_ramp;

% v(0) = 0
v_poly = polyint(a_poly, 0);

% d(0) = 0
d_poly = polyint(v_poly,0);

j_poly = 0;

% Add this rampup phase to path planning data object
% Format: add_rampup(hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
pp_data.add_rampup(1, 'y', t_ramp, d_poly, v_poly, a_poly, j_poly);

%% Throw-Catch Path Pre-Planning (Period I, II, III) - Hand1 - Y
d_top = workspace1_max_height;
v_top = 0;
a_top = 0;
eq_system = @(x) tc_system_solver(x, d_throw_y, v_throw_y, d_top, v_top, a_top, t_air);

a0_guess = -12;
a2_guess = -12;
t0_guess = 0.7/3;
t1_guess = 0.7/3;
t2_guess = 0.7/3;

x0 = [a0_guess, a2_guess, t0_guess, t1_guess, t2_guess];
options = optimset('MaxFunEvals',100000000,'maxIter',100000000);
x = fsolve(eq_system,x0,options);

% Make sure all time solutions are positive. 
for i = 3:5
    if x(i) < 0
        error('One or more time values is negative')
    end
end

% Extract Results
a0 = x(1);
a1 = a_top;
a2 = x(2);

t0 = x(3);
t1 = x(4);
t2 = x(5);

d0 = d_throw_y;
d1 = d_top;
d2 = d_top;
d3 = d_throw_y;

v0 = v_throw_y;
v1 = v_top;
v2 = v_top;
v3 = -v_throw_y;

% Double Check Results
check_tolerance = 0.05;
v_1 = v0 + a0*t0;
v_2 = v_1 + a1*t1;
v_final = v_2 + a2*t2;
d_1 = d0 + v0*t0 + 0.5*a0*t0^2;
d_2 = d_1 + v1*t1 + 0.5*a1*t1^2;
d_final = d_2 + v2*t2 + 0.5*a2*t2^2;

if ((v_final > (v3 + check_tolerance*abs(v3))) || (v_final < (v3 - check_tolerance*abs(v3))))
    error('Period tc velocity does not check out')
elseif ((d_final > (d3 + check_tolerance*abs(d3))) || (d_final < (d3 - check_tolerance*abs(d3))))
    error('Period tc velocity does not check out')
end

%% Period 1 Planning
% Throw point to top
% Initial: a0, v0, d0
% Final: a0, v1, d1

a_poly = a0;
v_poly = polyint(a_poly, v0);
d_poly = polyint(v_poly,d0);
j_poly = 0;

pp_data.add_loop(1, 'y', t0, d_poly, v_poly, a_poly, j_poly);

%% Period 2 Planning
% Rest at Top
% Initial: a1, v1, d1
% Final: a1, v2, d2

a_poly = a1;
v_poly = polyint(a_poly, v1);
d_poly = polyint(v_poly, d1);
j_poly = 0;

pp_data.add_loop(1, 'y', t1, d_poly, v_poly, a_poly, j_poly);

%% Period 3 Planning
% top to execute catch
% Initial: a2, v2, d2
% Final: a2, v3, d3

a_poly = a2;
v_poly = polyint(a_poly, v2);
d_poly = polyint(v_poly, d2);
j_poly = 0;

pp_data.add_loop(1, 'y', t2, d_poly, v_poly, a_poly, j_poly);

%% Catch-Throw Pre-Planning (Periods IV, V, VI)
d_bottom = workspace1_min_height;
v_bottom = 0;
a_bottom = 0;
eq_system = @(x) ct_system_solver(x, d_throw_y, v_throw_y, d_bottom, v_bottom, a_bottom, t_ct);

a0_guess = +12;
a2_guess = +12;
t0_guess = 0.7/3;
t1_guess = 0.7/3;
t2_guess = 0.7/3;

x0 = [a0_guess, a2_guess, t0_guess, t1_guess, t2_guess];
options = optimset('MaxFunEvals',100000000,'maxIter',100000000);
x = fsolve(eq_system,x0,options);

% Make sure all time solutions are positive. 
for i = 3:5
    if x(i) < 0
        error('One or more time values is negative')
    end
end

% Extract Results
cta0 = x(1);
cta1 = a_bottom;
cta2 = x(2);

ctt0 = x(3);
ctt1 = x(4);
ctt2 = x(5);

ctd0 = d_throw_y;
ctd1 = d_bottom;
ctd2 = d_bottom;
ctd3 = d_throw_y;

ctv0 = -v_throw_y;
ctv1 = v_bottom;
ctv2 = v_bottom;
ctv3 = v_throw_y;

% Double Check Results
check_tolerance = 0.05;
v_1 = ctv0 + cta0*ctt0;
v_2 = v_1 + cta1*ctt1;
v_final = v_2 + cta2*ctt2;
d_1 = ctd0 + ctv0*ctt0 + 0.5*cta0*ctt0^2;
d_2 = d_1 + ctv1*ctt1 + 0.5*cta1*ctt1^2;
d_final = d_2 + ctv2*ctt2 + 0.5*cta2*ctt2^2;

if ((v_final > (ctv3 + check_tolerance*abs(ctv3))) || (v_final < (ctv3 - check_tolerance*abs(ctv3))))
    error('Period tc velocity does not check out')
elseif ((d_final > (ctd3 + check_tolerance*abs(ctd3))) || (d_final < (ctd3 - check_tolerance*abs(ctd3))))
    error('Period tc velocity does not check out')
end

%% Period 4 Planning
% catch to rest at bottom
% Initial: dthrow, -vthrow, a0
% Final: 0,0,a0

a_poly = cta0;
v_poly = polyint(a_poly, ctv0);
d_poly = polyint(v_poly, ctd0);
j_poly = 0;

pp_data.add_loop(1, 'y', ctt0, d_poly, v_poly, a_poly, j_poly);

%% Period 5 Planning
% rest at bottom
a_poly = cta1;
v_poly = polyint(a_poly, ctv1);
d_poly = polyint(v_poly, ctd1);
j_poly = 0;

pp_data.add_loop(1, 'y', ctt1, d_poly, v_poly, a_poly, j_poly);

%% Period 6 Planning
% Rest at bottom to throw
a_poly = cta2;
v_poly = polyint(a_poly, ctv2);
d_poly = polyint(v_poly, ctd2);
j_poly = 0;

pp_data.add_loop(1, 'y', ctt2, d_poly, v_poly, a_poly, j_poly);

%% Ramp-down I, II, III, IV Planning
% These functions are equivalent to the looping I, II, and III, IV functions
% since the motion is a throw-catch motion. 
% We can simply copy these values

rampup_durations = pp_data.get_loop_durations(1, 'y');

% Period I
time = rampup_durations(1);
d_poly = pp_data.get_loop_dpoly(1, 'y', 1);
v_poly = pp_data.get_loop_vpoly(1, 'y', 1);
a_poly = pp_data.get_loop_apoly(1, 'y', 1);
j_poly = pp_data.get_loop_jpoly(1, 'y', 1);
% Add this data to rampdown I
pp_data.add_rampdown(1, 'y', time, d_poly, v_poly, a_poly, j_poly)

% Period II
time = rampup_durations(2);
d_poly = pp_data.get_loop_dpoly(1, 'y', 2);
v_poly = pp_data.get_loop_vpoly(1, 'y', 2);
a_poly = pp_data.get_loop_apoly(1, 'y', 2);
j_poly = pp_data.get_loop_jpoly(1, 'y', 2);
% Add this data to rampdown II
pp_data.add_rampdown(1, 'y', time, d_poly, v_poly, a_poly, j_poly)

% Period III
time = rampup_durations(3);
d_poly = pp_data.get_loop_dpoly(1, 'y', 3);
v_poly = pp_data.get_loop_vpoly(1, 'y', 3);
a_poly = pp_data.get_loop_apoly(1, 'y', 3);
j_poly = pp_data.get_loop_jpoly(1, 'y', 3);
% Add this data to rampdown III
pp_data.add_rampdown(1, 'y', time, d_poly, v_poly, a_poly, j_poly)

% Period IV
time = rampup_durations(4);
d_poly = pp_data.get_loop_dpoly(1, 'y', 4);
v_poly = pp_data.get_loop_vpoly(1, 'y', 4);
a_poly = pp_data.get_loop_apoly(1, 'y', 4);
j_poly = pp_data.get_loop_jpoly(1, 'y', 4);
% Add this data to rampdown IV
pp_data.add_rampdown(1, 'y', time, d_poly, v_poly, a_poly, j_poly)

%% Calculate the number of cycles to run
t_rampup = sum(pp_data.get_rampup_durations(1, 'y'));
t_loop = sum(pp_data.get_loop_durations(1, 'y'));
t_rampdown = sum(pp_data.get_loop_durations(1, 'y'));

t_overhead = t_rampup + t_rampdown;
cycles = floor((simulation_time - t_overhead) / t_loop);

if cycles < 0
    cycles = 1;
end

pp_data.set_cycles(cycles)
end

%% Helper Functions
function F = tc_system_solver(x, d_throw, v_throw, d_top, v_top, a_top, t_air)
d0 = d_throw;
d1 = d_top;
d2 = d_top;
d3 = d_throw;
v0 = v_throw;
v1 = v_top;
v2 = v_top;
v3 = -v_throw;
a1 = a_top;

a0 = x(1);
a2 = x(2);
t0 = x(3);
t1 = x(4);
t2 = x(5);

F(1) = d0 - d1 + v0*t0 + 0.5*a0*t0^2;
%F(2) = d1 - d2 + v1*t1 + 0.5*a1*t1^2;
F(2) = d2 - d3 + v2*t2 + 0.5*a2*t2^2;

F(3) = t0 + t1 + t2 - t_air;

F(4) = v0 - v1 + a0*t0;
%F(5) = v1 - v2 + a1*t1;
F(5) = v2 - v3 + a2*t2;

%F(7) = v0^2 - v1^2 + 2*a0*(d1-d0);
%F(7) = v1^2 - v2^2 + 2*a1*(d2 - d1);
%F(8) = v2^2 - v3^2 + 2*a2*(d3 - d2);
end

function F = ct_system_solver(x, d_throw, v_throw, d_bottom, v_bottom, a_bottom, t_ct)
d0 = d_throw;
d1 = d_bottom;
d2 = d_bottom;
d3 = d_throw;

v0 = -v_throw;
v1 = v_bottom;
v2 = v_bottom;
v3 = v_throw;

a1 = a_bottom;

a0 = x(1);
a2 = x(2);
t0 = x(3);
t1 = x(4);
t2 = x(5);

F(1) = d0 - d1 + v0*t0 + 0.5*a0*t0^2;
%F(2) = d1 - d2 + v1*t1 + 0.5*a1*t1^2;
F(2) = d2 - d3 + v2*t2 + 0.5*a2*t2^2;

F(3) = t0 + t1 + t2 - t_ct;

F(4) = v0 - v1 + a0*t0;
%F(5) = v1 - v2 + a1*t1;
F(5) = v2 - v3 + a2*t2;

%F(7) = v0^2 - v1^2 + 2*a0*(d1-d0);
%F(7) = v1^2 - v2^2 + 2*a1*(d2 - d1);
%F(8) = v2^2 - v3^2 + 2*a2*(d3 - d2);
end
