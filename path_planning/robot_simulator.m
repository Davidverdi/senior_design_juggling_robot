% mainscript.m
% David Verdi
% Created: 12/7/2016
% Updated: 1/25/2017

% This is the main script to run when simulating the juggling robot

%% Set Enviornmental Variables
% Chose path planning and path generating functions
path_planner = 'path_planning_level1_v10';
arm1_pathgen = 'arm1path_level1_v10';
arm2_pathgen = '';

t0 = 0;                          % Initial Time
timestep = 0.005;                 % Time Step
simulation_time = 1; %0.1 + 0.7555 + 0.7555; % Final Time

%% Execute Path Planning Function
pathplan_fun = str2func(path_planner);
path_data = pathplan_fun(simulation_time);

workspace1 = path_data.workspace1;
workspace2 = path_data.workspace2;

%% Setup Simulation Enviornment
simulation = phys_sim(t0, timestep, simulation_time, path_data);

simulation.set_hand_size([0.1, 0.05]) % [x_size, y_size] in meters
simulation.set_hand_color('blue')

simulation.set_ball_radius(0.03) % Radius in meters
simulation.set_ball_color('red')

simulation.set_throw_tolerance(0.01) % release if velocity/acceleration is <2% off
simulation.set_catch_tolerance(0.5)

simulation.add_workspace(workspace1)
simulation.add_workspace(workspace2)

%% Setup Hand 1 on Simulation
h1_id_number = 1; % Sequential integer number to identify objects
h1_intro_time = 0; % Time when hand1 first shows up on simulation
h1_type = 'hand'; % 'hand' or 'ball'
h1_initial_cond = [0 0 0 0 0 0 0 0];     % [x y vx vy ax ay jx jy] 
h1_bound_state = true; % If true, the object follows a path, or inherits the parent's path. If false, it is in freefall. 
h1_parent_id = []; % Either [] or the id_number of the parent to inherit the path from
h1_bound_function = str2func(arm1_pathgen); % The path generating function. Use str2func to turn the string into a function.

simulation.add_obj(h1_id_number, h1_intro_time, h1_type, h1_initial_cond, h1_bound_state, h1_parent_id, h1_bound_function)  

%% Setup Ball 1 on Simulation
b1_id_number = 2;
b1_intro_time = 0;
b1_type = 'ball';
b1_initial_cond = [0 0 0 0 0 0 0 0];
b1_bound_state = true;
b1_parent_id = 1;
b1_bound_function = [];

simulation.add_obj(b1_id_number, b1_intro_time, b1_type, b1_initial_cond, b1_bound_state, b1_parent_id, b1_bound_function)  

%% Setup a Second Ball for Debugging
% This second ball has the correct d_throw and v_throw but is in freefall,
% for comparison
b2_id_number = 3;
b2_intro_time = 1;
b2_type = 'ball';
b2_initial_cond = [0.2 0.4 0 3.7059 0 -9.81 0 0];
b2_bound_state = false;
b2_parent_id = [];
b2_bound_function = [];

%simulation.add_obj(b2_id_number, b2_intro_time, b2_type, b2_initial_cond, b2_bound_state, b2_parent_id, b2_bound_function) 

%% Run Siumulation
simulation.run_simulation()


% %% Simulation
% % For Refrence: phys_obj(ini_cond, time0, timestep, shape, size, color, bound_state, bound_state_function, bound_state_data)
% hand1  = phys_obj(h1_initial_cond, t0, timestep, h1_shape, h1_size, h1_color, h1_bound_state, h1_pgfun, path_data);
% 
% 
% for t = t0:timestep:sim_time
%     clf
%     hold off
%     axis([-1.3 1.3 -1.3 1.3])
%     hand1.update()
%     hand1.draw()
%     str = strcat('time = ',num2str(t));
%     text(-1,-1,str)
%     pause(timestep)
% end

