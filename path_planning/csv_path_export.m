% csv_path_export.m
% David Verdi
% Created: 1/30/2017
% Updated: 3/22/2017

% Exports the selected path planning ACCELERATIONS to a CSV file. 
% CSV file format is TIME, ACCELERATION for each row. 
% There are no column headers. 

%% Set Functions and Time Range NO DELAY
clear
t_initial = 0;
t_final = 2;
resolution = 0.001;
path_planner = 'path_planning_level1_v10';
path_gen = 'arm1path_level1_v10';
r_pulley = 0.06522878/2; %meters

filename_angle_y = 'nodelay_Y_path_angle_csv.csv';
filename_ang_vel_y = 'nodelay_Y_path_angle_vel_csv.csv';
filename_ang_accel_y = 'nodelay_Y_path_angle_accel_csv.csv';
filename_pos_y = 'nodelay_Y_path_position_csv.csv';
filename_vel_y = 'nodelay_Y_path_vel_csv.csv';
filename_accel_y = 'nodelay_Y_path_accel_csv.csv';
filename_angle_x = 'nodelay_x_path_angle_csv.csv';
filename_ang_vel_x = 'nodelay_x_path_angle_vel_csv.csv';
filename_ang_accel_x = 'nodelay_x_path_angle_accel_csv.csv';
filename_pos_x = 'nodelay_x_path_position_csv.csv';
filename_vel_x = 'nodelay_x_path_vel_csv.csv';
filename_accel_x = 'nodelay_x_path_accel_csv.csv';


%% Create Function Handles
path_plan_fun = str2func(path_planner);
path_gen_fun = str2func(path_gen);
path_plan_data = path_plan_fun(t_final);

%% Get workspace data to offset the numbers for solidworks:
workspace_no_delay = path_plan_data.workspace1;
workspace_delay = path_plan_data.workspace2;

workspace_width = workspace_no_delay(2) - workspace_no_delay(1) %workspace 1 max minus min
workspace_gap = 2*workspace_no_delay(1)

%% Loop
t = t_initial:resolution:(t_final);
for i = 1:length(t)
    [x, y, v_x, v_y, a_x, a_y, j_x, j_y] = path_gen_fun(path_plan_data, t(i));
    ay(i) = a_y;
    vy(i) = v_y;
    y_(i) = y;
    ax(i) = a_x;
    vx(i) = v_x;
    x_(i) = x - (workspace_gap/2 + workspace_width/2);
end
angle = (180/pi)*y_/r_pulley;
ang_accel = (180/pi)*ay/r_pulley;
ang_vel = (180/pi)*vy/r_pulley;
time_positionangle_matrix = [t', angle'];
time_position_matrix = [t', y_'];
time_angvel_matrix = [t', ang_vel'];
time_vel_matrix = [t', vy'];
time_accel_matrix = [t', ay'];
time_angaccel_matrix = [t', ang_accel'];

csvwrite(filename_pos_y, time_position_matrix)
csvwrite(filename_vel_y, time_vel_matrix)
%csvwrite(filename_accel_y, time_accel_matrix)

%csvwrite(filename_angle_y, time_positionangle_matrix)
%csvwrite(filename_ang_vel_y, time_angvel_matrix)
%csvwrite(filename_ang_accel_y, time_angaccel_matrix)

% Now do x
angle = (180/pi)*x_/r_pulley;
ang_accel = (180/pi)*ax/r_pulley;
ang_vel = (180/pi)*vx/r_pulley;
time_positionangle_matrix = [t', angle'];
time_position_matrix = [t', x_'];
time_angvel_matrix = [t', ang_vel'];
time_vel_matrix = [t', vx'];
time_accel_matrix = [t', ax'];
time_angaccel_matrix = [t', ang_accel'];

%csvwrite(filename_pos_x, time_position_matrix)
%csvwrite(filename_vel_x, time_vel_matrix)
%csvwrite(filename_accel_x, time_accel_matrix)

%csvwrite(filename_angle_x, time_positionangle_matrix)
%csvwrite(filename_ang_vel_x, time_angvel_matrix)
%csvwrite(filename_ang_accel_x, time_angaccel_matrix)




%% Set Functions and Time Range
filename_angle_y = 'delay_Y_path_angle_csv.csv';
filename_ang_vel_y = 'delay_Y_path_angle_vel_csv.csv';
filename_ang_accel_y = 'delay_Y_path_angle_accel_csv.csv';
filename_pos_y = 'delay_Y_path_position_csv.csv';
filename_vel_y = 'delay_Y_path_vel_csv.csv';
filename_accel_y = 'delay_Y_path_accel_csv.csv';
filename_angle_x = 'delay_x_path_angle_csv.csv';
filename_ang_vel_x = 'delay_x_path_angle_vel_csv.csv';
filename_ang_accel_x = 'delay_x_path_angle_accel_csv.csv';
filename_pos_x = 'delay_x_path_position_csv.csv';
filename_vel_x = 'delay_x_path_vel_csv.csv';
filename_accel_x = 'delay_x_path_accel_csv.csv';

%path_gen = 'arm2path_level2_v01';


%% Create Function Handles
path_gen_fun = str2func(path_gen);

%% Loop
t = t_initial:resolution:(t_final);
for i = 1:length(t)
    [x, y, v_x, v_y, a_x, a_y, j_x, j_y] = path_gen_fun(path_plan_data, t(i));
    ay(i) = a_y;
    vy(i) = v_y;
    y_(i) = y;
    ax(i) = a_x;
    vx(i) = v_x;
    x_(i) = x - (workspace_gap/2 + workspace_width/2);
end
angle = (180/pi)*y_/r_pulley;
ang_accel = (180/pi)*ay/r_pulley;
ang_vel = (180/pi)*vy/r_pulley;
time_positionangle_matrix = [t', angle']
time_position_matrix = [t', y_']
time_angvel_matrix = [t', ang_vel'];
time_vel_matrix = [t', vy'];
time_accel_matrix = [t', ay'];
time_angaccel_matrix = [t', ang_accel'];

%csvwrite(filename_pos_y, time_position_matrix)
%csvwrite(filename_vel_y, time_vel_matrix)
%csvwrite(filename_accel_y, time_accel_matrix)

%csvwrite(filename_angle_y, time_positionangle_matrix)
%csvwrite(filename_ang_vel_y, time_angvel_matrix)
%csvwrite(filename_ang_accel_y, time_angaccel_matrix)


% Now do x
angle = (180/pi)*x_/r_pulley;
ang_accel = (180/pi)*ax/r_pulley;
ang_vel = (180/pi)*vx/r_pulley;
time_positionangle_matrix = [t', angle']
time_position_matrix = [t', x_']
time_angvel_matrix = [t', ang_vel'];
time_vel_matrix = [t', vx'];
time_accel_matrix = [t', ax'];
time_angaccel_matrix = [t', ang_accel'];

%csvwrite(filename_pos_x, time_position_matrix)
%csvwrite(filename_vel_x, time_vel_matrix)
%csvwrite(filename_accel_x, time_accel_matrix)

%csvwrite(filename_angle, time_positionangle_matrix)
%csvwrite(filename_ang_vel, time_angvel_matrix)
%csvwrite(filename_ang_accel, time_angaccel_matrix)

