function pp_data = path_planning_level2_v02(simulation_time)
% path_planning_level2_v01.m
% Jamar Liburd
% Created: 2/15/2017
% Updated: 3/22/2017 (DV)

% WORKS WITH: arm1path_level2_v01.m

% LEVEL TWO OF SUCCESS:
% (With some elements of level three)
% Hand 1 throws the ball up, then goes to catch position, then goes home.
% Hand 2 delays, then throws ball up, the goes to catch position, then goes
% home. 

% The X distances and velocities follow a polynomial. 

% This function calculates a constrained path for level 2 of success. 
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
workspace_width = 0.35; %m
workspace_height = 0.55; %m
workspace_gap = 0.40; %m

% Set Ball Trajectory Data
ball_trajectory_height = 0.6; %(0.6)m
d_throw_y = workspace_height/2; %m
t_ct = 1; %s time between catch-throw
%NOTE: lag time is buried because it depends on t_ramp_y

% Calculate trajectory of ball based on inputs
v_throw_y = sqrt(2 * 9.81 * ball_trajectory_height);
t_air = (2 * v_throw_y)/9.81;
d_trajectory = workspace_gap + workspace_width;
v_throw_x = (d_trajectory)/t_air;

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

%% Ramp-up Path Planning- Hand1 (No Delay) - Y
% The ramp-up contains one phase: An acceleration of a_ramp.
% a(0) = a_ramp   a(t_ramp) = a_ramp
% v(0) = 0        v(t_ramp) = v_throw
% d(0) = 0        d(t_ramp) = d_throw

% v_f^2 = v_0^2 + 2*a*d
% a = (v_f^2)/(2*d)
a_ramp = (v_throw_y^2)/(2*d_throw_y);

% v_f = v_0 + a*t
% t = v_f/a
t_ramp_y = v_throw_y/a_ramp;

a_poly = a_ramp;

% v(0) = 0
v_poly = polyint(a_poly, 0);

% d(0) = 0
d_poly = polyint(v_poly,0);

j_poly = 0;

% Add this rampup phase to path planning data object
% Format: add_rampup(hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
pp_data.add_rampup(1, 'y', t_ramp_y, d_poly, v_poly, a_poly, j_poly);

%% Ramp-up Path Planning- Hand1 (No Delay) - X 
% Hand1 is the hand with no delay. There is one polynomial rampup phase,
% where the ball goes from the home position (at the bottom middle) to the
% throw point, at the throw height.
% The following rules are used to constrain the path:
% d(t_0) = hand1_home_x    
% v(t_0) = 0                 v(t_ramp_y) = v_throw
% a(t_0) = 0                 a(t_ramp_y) = 0
% There are five constraints, so a fourth degree polynomial is used

t_0 = 0;
d_0 = hand1_home_x;
v_0 = 0;
a_0 = 0;

M = [t_0^4  t_0^3  t_0^2  t_0  1;
      4*t_0^3  3*t_0^2  2*t_0  1  0;
     12*t_0^2 6*t_0 2 0 0;
     4*t_ramp_y^3  3*t_ramp_y^2  2*t_ramp_y  1  0;
     12*t_ramp_y^2  6*t_ramp_y  2  0  0];

b = [d_0 v_0 a_0 -v_throw_x a_0]';

% Solve M*a = b
d_poly = (M\b)';
v_poly = polyder(d_poly);
a_poly = polyder(v_poly);
j_poly = polyder(a_poly);
pp_data.add_rampup(1, 'x', t_ramp_y, d_poly, v_poly, a_poly, j_poly);
disp('Hand 1 d_throw (No delay)')
d_throw_x = polyval(d_poly, t_ramp_y)

%% Throw-Catch Path Pre-Planning (Period I, II, III) For Y - Hand1 (No Delay)
d_top = workspace1_max_height;
v_top = 0;
a_top = 0;
lag_time = 2*t_ramp_y; %arbitrarily chosen
% VERY IMPORTANT THIS IS A + FOR HAND1
t_air_hand1 = t_air + lag_time;
eq_system = @(x) tc_system_solver(x, d_throw_y, v_throw_y, d_top, v_top, a_top, t_air_hand1);

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

%% Period 1 Planning For Y - Hand1 (No delay)
% Throw point to top
% Initial: a0, v0, d0
% Final: a0, v1, d1

a_poly = a0;
v_poly = polyint(a_poly, v0);
d_poly = polyint(v_poly,d0);
j_poly = 0;

pp_data.add_rampup(1, 'y', t0, d_poly, v_poly, a_poly, j_poly);

%% Period 2 Planning For Y - Hand1 (No delay)
% Rest at Top
% Initial: a1, v1, d1
% Final: a1, v2, d2

a_poly = a1;
v_poly = polyint(a_poly, v1);
d_poly = polyint(v_poly, d1);
j_poly = 0;

pp_data.add_rampup(1, 'y', t1, d_poly, v_poly, a_poly, j_poly);

%% Period 3 Planning For Y - Hand1 (No delay)
% top to execute catch
% Initial: a2, v2, d2
% Final: a2, v3, d3

a_poly = a2;
v_poly = polyint(a_poly, v2);
d_poly = polyint(v_poly, d2);
j_poly = 0;

pp_data.add_rampup(1, 'y', t2, d_poly, v_poly, a_poly, j_poly);

%% Throw-Catch Path Planning for For X - Hand1 (No delay)
% This consists of a throw to edge path (profiled jerk), a edge to catch 
% path (Polynomial), and a catch to edge path (Profiled jerk). 
%% Throw to Edge Path Planning for X - Hand1 (No delay):
% First, we have to get the hand to go from d_throw_x to go to the edge of
% the workspace, using a constant jerk. The conditions for this are:
% d(0) = d_throw_x
% v(0) = -v_throw_x
% a(0) = 0

% d(t = ??) = workspace1_min_x
% v(t = ??) = 0
% a(t = ??) = 0

% List Knowns
d_throw = d_throw_x;
v_throw = -v_throw_x;
a_throw = 0;
d_edge = workspace1_min_x;
v_edge = 0;
a_edge = 0;

% Guess Unknowns
j1_guess = 0.2;
t1_guess = (t_air + t_ramp_y)/4;

x0 = [j1_guess, t1_guess];

eq_system = @(x) x1_tc_system_solver(x, d_throw, v_throw, a_throw, d_edge, v_edge, a_edge);

options = optimset('MaxFunEvals',100000,'maxIter',100000);

x = fsolve(eq_system,x0,options);

%Extract Results
j1 = x(1);
t1 = x(2);
v0 = -v_throw_x;
d0 = d_throw_x;

a0 = a_throw;
a_edge = a0 + j1*t1;
v_edge = v0 + a0*t1 + 0.5*j1*t1^2;
d_edge = d0 + v0*t1 + 0.5*a0*t1^2 + (1/6)*j1*t1^3;
t_tc_remaining = t_air_hand1 - t1;

% disp('t1')
% disp(t1)
% disp('t_air_hand1')
% disp(t_air_hand1)
% disp('t_tc_remaining')
% disp(t_tc_remaining)


j_poly = j1;
a_poly = polyint(j_poly, a0);
v_poly = polyint(a_poly, v0);
d_poly = polyint(v_poly, d0);

pp_data.add_rampup(1, 'x', t1, d_poly, v_poly, a_poly, j_poly);

%% Edge to Catch Path Planning - Hand1, X (right)
% d(0) = workspace1_min_x;
% v(0) = 0;
% a(0) = a_edge;
% 
% Mirror throw distance across midpoint of workspace)
% d(t_tc_remaining) = workspace_max_x - (d_throw_x - workspace_min_x)
% v(t_tc_remaining) = v_throw_x
% a(t_tc_remaining) = 0

% Six constraints, so this is a fifth order polynomial. 
t_0 = 0;
d_0 = workspace1_min_x;
v_0 = v_edge;
a_0 = a_edge;

t_catch = t_tc_remaining;
disp('Hand 1 d_catch (no delay)')
d_catch = workspace1_max_x - (d_throw_x - workspace1_min_x)
v_catch = v_throw_x;
a_catch = 0;

M = [t_0^5 t_0^4  t_0^3  t_0^2  t_0  1;
     5*t_0^4 4*t_0^3  3*t_0^2  2*t_0  1  0;
     20*t_0^3  12*t_0^2 6*t_0 2 0 0;
     t_catch^5  t_catch^4  t_catch^3  t_catch^2  t_catch  1;
     5*t_catch^4  4*t_catch^3  3*t_catch^2  2*t_catch  1  0;
     20*t_catch^3  12*t_catch^2 6*t_catch 2 0 0];

b = [d_0 v_0 a_0 d_catch v_catch a_catch]';

% Solve M*a = b
% M\b is the same as inv(M)*b, and it gives us the coefficients of the
% distance polynomial. 
d_poly = (M\b)';
v_poly = polyder(d_poly);
a_poly = polyder(v_poly);
j_poly = polyder(a_poly);

% figure(6)
% plot_time = 0:0.001:t_tc_remaining;
% plot(d_poly, polyval(d_poly,plot_time));

pp_data.add_rampup(1, 'x', t_tc_remaining, d_poly, v_poly, a_poly, j_poly);

%% Catch to Edge Path Planning - Hand1, X (right)
% d(0) = d_catch
% v(0) = v_catch
% a(0) = a_catch

% d(t = ??) = workspace1_max_x
% v(t = ??) = 0
% a(t = ??) = 0

% List Knowns
d_catch;
v_catch = v_throw_x;
a_catch;
d_edge = workspace1_max_x;
v_edge = 0;
a_edge = 0;

% Guess Unknowns
j1_guess = -10;
t1_guess = (t_air + t_ramp_y)/4;

x0 = [j1_guess, t1_guess];

eq_system = @(x) x1_tc_system_solver(x, d_catch, v_catch, a_catch, d_edge, v_edge, a_edge);

options = optimset('MaxFunEvals',100000,'maxIter',100000);

x = fsolve(eq_system,x0,options);

%Extract solutions:
j1 = x(1);
t1 = x(2);
a0 = a_catch;
v0 = v_catch;
d0 = d_catch;


j_poly = j1;
a_poly = polyint(j_poly, a0);
v_poly = polyint(a_poly, v0);
d_poly = polyint(v_poly, d0);

% plot_time = 0:0.001:t1;
% figure(4)
% plot(plot_time, polyval(d_poly,plot_time))

pp_data.add_rampup(1, 'x', t1, d_poly, v_poly, a_poly, j_poly);


%% Catch-Throw Pre-Planning (Periods IV, V, VI) - Y, Hand1 (Right)
% In this version we only do catch to rest. 
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

%% Period 4 Planning - Y, Hand1 (Right)
% catch to rest at bottom
% Initial: dthrow, -vthrow, a0
% Final: 0,0,a0

a_poly = cta0;
v_poly = polyint(a_poly, ctv0);
d_poly = polyint(v_poly, ctd0);
j_poly = 0;

pp_data.add_rampdown(1, 'y', ctt0, d_poly, v_poly, a_poly, j_poly);
% We discard the rest, because it is irrelevant for level two/three of
% success. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%                  HAND II                    %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Hand II Ramp-up phase I (delay) for  Y
% The rampup for hand2 has a phase where all positoins, acclerations, and
% velocities are zero, so that hand1 can throw the ball.
lag_time = 2*t_ramp_y;
a_poly = [0];
v_poly = [0];
d_poly = [0];
j_poly = [0];

pp_data.add_rampup(2, 'y', lag_time, d_poly, v_poly, a_poly, j_poly);
%% Hand2 Rampup Phase II
% second phase: An acceleration of a_ramp.
% a(0) = a_ramp   a(t_ramp) = a_ramp
% v(0) = 0        v(t_ramp) = v_throw
% d(0) = 0        d(t_ramp) = d_throw

% v_f^2 = v_0^2 + 2*a*d
% a = (v_f^2)/(2*d)
a_ramp = (v_throw_y^2)/(2*d_throw_y);

% v_f = v_0 + a*t
% t = v_f/a
t_ramp_y = v_throw_y/a_ramp;

a_poly = a_ramp;

% v(0) = 0
v_poly = polyint(a_poly, 0);

% d(0) = 0
d_poly = polyint(v_poly,0);

j_poly = 0;

% Add this rampup phase to path planning data object
% Format: add_rampup(hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
pp_data.add_rampup(2, 'y', t_ramp_y, d_poly, v_poly, a_poly, j_poly);

%% Hand II Rampup Delay for X
% The rampup for hand2 has a phase where all positoins, acclerations, and
% velocities are zero, so that hand1 can throw the ball.
lag_time = 2*t_ramp_y;
a_poly = [0];
v_poly = [0];
d_poly = hand1_home_x;  %CHANGE WHEN YOU MIRROR IT
j_poly = [0];

pp_data.add_rampup(2, 'x', lag_time, d_poly, v_poly, a_poly, j_poly);

%% Ramp-up Path Planning- Hand2 (Delay) - X 
% Hand1 is the hand with no delay. There is one polynomial rampup phase,
% where the ball goes from the home position (at the bottom middle) to the
% throw point, at the throw height.
% The following rules are used to constrain the path:
% d(t_0) = hand1_home_x    
% v(t_0) = 0                 v(t_ramp_y) = -v_throw
% a(t_0) = 0                 a(t_ramp_y) = 0
% There are five constraints, so a fourth degree polynomial is used

t_0 = 0;
d_0 = hand1_home_x;
v_0 = 0;
a_0 = 0;

M = [t_0^4  t_0^3  t_0^2  t_0  1;
      4*t_0^3  3*t_0^2  2*t_0  1  0;
     12*t_0^2 6*t_0 2 0 0;
     4*t_ramp_y^3  3*t_ramp_y^2  2*t_ramp_y  1  0;
     12*t_ramp_y^2  6*t_ramp_y  2  0  0];

b = [d_0 v_0 a_0 -v_throw_x a_0]';

% Solve M*a = b
d_poly = (M\b)';
v_poly = polyder(d_poly);
a_poly = polyder(v_poly);
pp_data.add_rampup(2, 'x', t_ramp_y, d_poly, v_poly, a_poly, j_poly);

disp('hand 2 d_throw (delay)')
d_throw_x = polyval(d_poly, t_ramp_y);


%% Throw-Catch Path Pre-Planning (Period I, II, III) - Hand2 - Y
d_top = workspace1_max_height;
v_top = 0;
a_top = 0;
%THIS IS A MINUS FOR HAND2
t_air_hand2 = t_air - lag_time;
eq_system = @(x) tc_system_solver(x, d_throw_y, v_throw_y, d_top, v_top, a_top, t_air_hand2);

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

% % Double Check Results
% check_tolerance = 0.05;
% v_1 = v0 + a0*t0;
% v_2 = v_1 + a1*t1;
% v_final = v_2 + a2*t2;
% d_1 = d0 + v0*t0 + 0.5*a0*t0^2;
% d_2 = d_1 + v1*t1 + 0.5*a1*t1^2;
% d_final = d_2 + v2*t2 + 0.5*a2*t2^2;
% 
% if ((v_final > (v3 + check_tolerance*abs(v3))) || (v_final < (v3 - check_tolerance*abs(v3))))
%     error('Period tc velocity does not check out')
% elseif ((d_final > (d3 + check_tolerance*abs(d3))) || (d_final < (d3 - check_tolerance*abs(d3))))
%     error('Period tc velocity does not check out')
% end

%% Period 1 Planning - Hand2 Y
% Throw point to top
% Initial: a0, v0, d0
% Final: a0, v1, d1

a_poly = a0;
v_poly = polyint(a_poly, v0);
d_poly = polyint(v_poly,d0);
j_poly = 0;

pp_data.add_rampup(2, 'y', t0, d_poly, v_poly, a_poly, j_poly);

%% Period 2 Planning - Hand2 Y
% Rest at Top
% Initial: a1, v1, d1
% Final: a1, v2, d2

a_poly = a1;
v_poly = polyint(a_poly, v1);
d_poly = polyint(v_poly, d1);
j_poly = 0;

pp_data.add_rampup(2, 'y', t1, d_poly, v_poly, a_poly, j_poly);

%% Period 3 Planning - Hand2 Y
% top to execute catch
% Initial: a2, v2, d2
% Final: a2, v3, d3

a_poly = a2;
v_poly = polyint(a_poly, v2);
d_poly = polyint(v_poly, d2);
j_poly = 0;

pp_data.add_rampup(2, 'y', t2, d_poly, v_poly, a_poly, j_poly);

%% Throw-Catch Path Planning for For X - Hand2 (left side)
% This consists of a throw to edge path (profiled jerk), a edge to catch 
% path (Polynomial), and a catch to edge path (Profiled jerk).

%% Throw to Edge Path Planning for X - Hand2 (left side)
% First, we have to get the hand to go from d_throw_x to go to the edge of
% the workspace, using a constant jerk. The conditions for this are:
% d(0) = d_throw_x
% v(0) = -v_throw_x
% a(0) = 0

% d(t = ??) = workspace1_min_x
% v(t = ??) = 0
% a(t = ??) = 0

% List Knowns
d_throw = d_throw_x;
v_throw = -v_throw_x;
a_throw = 0;
d_edge = workspace1_min_x;
v_edge = 0;
a_edge = 0;

% Guess Unknowns
j1_guess = 0.2;
t1_guess = (t_air + t_ramp_y)/4;

x0 = [j1_guess, t1_guess];

eq_system = @(x) x1_tc_system_solver(x, d_throw, v_throw, a_throw, d_edge, v_edge, a_edge);

options = optimset('MaxFunEvals',100000,'maxIter',100000);

x = fsolve(eq_system,x0,options);

%Extract Results
j1 = x(1);
t1 = x(2);
v0 = -v_throw_x;
d0 = d_throw_x;

a0 = a_throw;
a_edge = a0 + j1*t1;
v_edge = v0 + a0*t1 + 0.5*j1*t1^2;
d_edge = d0 + v0*t1 + 0.5*a0*t1^2 + (1/6)*j1*t1^3;
t_tc_remaining = t_air_hand2 - t1;

j_poly = j1;
a_poly = polyint(j_poly, a0);
v_poly = polyint(a_poly, v0);
d_poly = polyint(v_poly, d0);

pp_data.add_rampup(2, 'x', t1, d_poly, v_poly, a_poly, j_poly);

%% Edge to Catch Path Planning -Hand2 (left side)
% d(0) = workspace1_min_x;
% v(0) = 0;
% a(0) = a_edge;
% 
% Mirror throw distance across midpoint of workspace)
% d(t_tc_remaining) = workspace_max_x - (d_throw_x - workspace_min_x)
% v(t_tc_remaining) = v_throw_x
% a(t_tc_remaining) = 0

% Six constraints, so this is a fifth order polynomial. 
t_0 = 0;
d_0 = workspace1_min_x;
v_0 = v_edge;
a_0 = a_edge;

t_catch = t_tc_remaining;
disp('d_catch hand 2 delay')
d_catch = workspace1_max_x - (d_throw_x - workspace1_min_x)
v_catch = v_throw_x;
a_catch = 0;

M = [t_0^5 t_0^4  t_0^3  t_0^2  t_0  1;
     5*t_0^4 4*t_0^3  3*t_0^2  2*t_0  1  0;
     20*t_0^3  12*t_0^2 6*t_0 2 0 0;
     t_catch^5  t_catch^4  t_catch^3  t_catch^2  t_catch  1;
     5*t_catch^4  4*t_catch^3  3*t_catch^2  2*t_catch  1  0;
     20*t_catch^3  12*t_catch^2 6*t_catch 2 0 0];

b = [d_0 v_0 a_0 d_catch v_catch a_catch]';

% Solve M*a = b
% M\b is the same as inv(M)*b, and it gives us the coefficients of the
% distance polynomial. 
d_poly = (M\b)';
v_poly = polyder(d_poly);
a_poly = polyder(v_poly);
j_poly = polyder(a_poly);

pp_data.add_rampup(2, 'x', t_tc_remaining, d_poly, v_poly, a_poly, j_poly);

%% Catch to Edge Path Planning -  Hand2 (left side) X
% d(0) = d_catch
% v(0) = v_catch
% a(0) = a_catch

% d(t = ??) = workspace1_max_x
% v(t = ??) = 0
% a(t = ??) = 0

% List Knowns
d_catch;
v_catch = v_throw_x;
a_catch;
d_edge = workspace1_max_x;
v_edge = 0;
a_edge = 0;

% Guess Unknowns
j1_guess = -10;
t1_guess = (t_air + t_ramp_y)/4;

x0 = [j1_guess, t1_guess];

eq_system = @(x) x1_tc_system_solver(x, d_catch, v_catch, a_catch, d_edge, v_edge, a_edge);

options = optimset('MaxFunEvals',100000,'maxIter',100000);

x = fsolve(eq_system,x0,options);

%Extract solutions:
j1 = x(1);
t1 = x(2);
a0 = a_catch;
v0 = v_catch;
d0 = d_catch;


j_poly = j1;
a_poly = polyint(j_poly, a0);
v_poly = polyint(a_poly, v0);
d_poly = polyint(v_poly, d0);

pp_data.add_rampup(2, 'x', t1, d_poly, v_poly, a_poly, j_poly);


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

% % Double Check Results
% check_tolerance = 0.05;
% v_1 = ctv0 + cta0*ctt0;
% v_2 = v_1 + cta1*ctt1;
% v_final = v_2 + cta2*ctt2;
% d_1 = ctd0 + ctv0*ctt0 + 0.5*cta0*ctt0^2;
% d_2 = d_1 + ctv1*ctt1 + 0.5*cta1*ctt1^2;
% d_final = d_2 + ctv2*ctt2 + 0.5*cta2*ctt2^2;
% 
% if ((v_final > (ctv3 + check_tolerance*abs(ctv3))) || (v_final < (ctv3 - check_tolerance*abs(ctv3))))
%     error('Period tc velocity does not check out')
% elseif ((d_final > (ctd3 + check_tolerance*abs(ctd3))) || (d_final < (ctd3 - check_tolerance*abs(ctd3))))
%     error('Period tc velocity does not check out')
% end

%% Period 4 Planning
% catch to rest at bottom
% Initial: dthrow, -vthrow, a0
% Final: 0,0,a0

a_poly = cta0;
v_poly = polyint(a_poly, ctv0);
d_poly = polyint(v_poly, ctd0);
j_poly = 0;

pp_data.add_rampdown(2, 'y', ctt0, d_poly, v_poly, a_poly, j_poly);

%% Calculate the number of cycles to run
% In level 3-4 there are no cycles. 
cycles = 0;
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
%%
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

function F = x1_tc_system_solver(x, d_throw, v_throw, a_throw, d_edge, v_edge, a_edge)
d0 = d_throw;
v0 = v_throw;   
a0 = a_throw;

d1 = d_edge;
v1 = v_edge;   
%a1 = a_edge;

% Extract guesses
j1 = x(1);
t1 = x(2);

% Velocity Equations
F(1) = v0 + a0*t1 + 0.5*j1*t1^2 - v1;

% Distance Equations
F(2) = d0 + v0*t1 + 0.5*a0*t1^2 + (1/6)*j1*t1^3 - d1;
end
