classdef phys_sim < handle
    % simulation.m
    % This is the phys_sim (physical simulation) class, which will contain 
    % all the constituent physical objects of the robot.
    % This simulation will produce all animations, contain all
    % physical objects, handle all object collisions, etc.  
    %
    % David Verdi
    % Created: 12/7/2016
    % Updated: 1/13/2017
     
    properties
        % Simulation Parameters
        time0
        time
        timestep
        simulation_time
        path_plan_data
        
        % Impact properties
        throw_tolerance = 0.01 %Default 1%
        catch_tolerance = 0.02 %Default 2%
        ball_restitution_coeff = 0.2
        
        % Simulation Visualization Settings
        ball_size = 0.03; % radius in m
        hand_size = [0.1,0.05]; %[x_size y_size]
        ball_color = 'red';
        hand_color = 'blue';
        
        workspace_buffer = 1.2; % 120% Excess around workspace plots
        workspace_color = [0.91 0.91 0.91];
        plot_size %[min_x max_x min_y max_y]
        
        
        
        % workspace_data is a cell array of of workspace bounding boxes.
        % Currently there is no detection for going out of the workspace
        % It is formatted as follows:
        % { [min_x max_x min_y max_y]
        %   [min_x max_x min_y max_y] }
        workspace_list
        
        
        % object_list is a cell array of objects in the simulation. It
        % behaves like a simple list
        object_list
        
        % object_hold is the cell array of objects that have not been
        % introduced yet into the simulation. Their t_0 hasn't come up yet
        object_hold
    end
    
    methods
        % constructor
        function this = phys_sim(t_0, t_step, sim_time, path_plan_data)
            this.time0 = t_0;
            this.timestep = t_step;
            this.simulation_time = sim_time;
            this.path_plan_data = path_plan_data;
            this.workspace_list = {};
            this.object_list = {};
            this.object_hold  = {};
        end
        
        % Setters
        function set_hand_size(this, hand_size)
            % Set hand size in meters [x_size y_size]
            this.hand_size = hand_size;
        end
        
        function set_ball_radius(this, ball_radius)
            % set ball radius in meters
            this.ball_size = ball_radius;
        end
        
        function set_hand_color(this, hand_color)
            % Default is 'blue'
            this.hand_color = hand_color;
        end
        
        function set_ball_color(this, ball_color)
            % Default is 'red'
            this.ball_color = ball_color;
        end
        
        function set_throw_tolerance(this, tolerance)
            % Default is 5%
            this.throw_tolerance = tolerance;
        end
        
        function set_catch_tolerance(this, tolerance)
            % Default is 5%
            this.catch_tolerance = tolerance;
        end
        
        % Workspace Adder
        function add_workspace(this, workspace_bb)
            % Takes in Workspace bounding box
            % [min_x max_x min_y max_y] in meters
            if ~ isempty(workspace_bb)
                this.workspace_list{end + 1} = workspace_bb;
            end
        end
        
        % Object Adder
        function add_obj(this, id_number, intro_time, type, ini_cond, bound_state, parent_id, bound_state_function)
            % id_number is a sequential identification number given to each
            % object in the simulation
            % type is a string, either 'hand' or 'ball'
            % ini_cond is the vector of initial conditions:
            % [x y vx vy ax ay jx jy]
            % Bound state is true or false
            % parent_id is the id number of the object that this object
            % will inherit the bound state function from. Otherwise it is
            % blank []
            % the bound state function is the function that the object
            % follows in the bound state. An object can have a parent_id,
            % or a bound_state_function, but not both
            if strcmp(type, 'hand')
                size = this.hand_size;
                color = this.hand_color;
                shape = 'rectangle';
            end
            
            if strcmp(type, 'ball')
                size = this.ball_size;
                color = this.ball_color;
                shape = 'circle';
            end
                
            if intro_time > this.time0
                this.object_hold{end + 1} = phys_obj(id_number, ini_cond, intro_time, this.timestep, shape, size, color, bound_state, parent_id, bound_state_function, this.path_plan_data);
            else 
                this.object_list{id_number} = phys_obj(id_number, ini_cond, intro_time, this.timestep, shape, size, color, bound_state, parent_id, bound_state_function, this.path_plan_data);
            end
        end
        
        function run_simulation(this)
            this.set_plot_size()
            figure(1)
            for t = this.time0 : this.timestep : this.simulation_time
                this.time = t;
                clf
                axis(this.plot_size)
                hold on
                this.plot_workspace()
                this.introduce_objects()
                this.resolve_bound_states()
                this.update_all()
                this.draw_all()
                this.draw_info()
                pause(this.timestep)
            end 
        end
        
        function update_all(this)
            % This function jumps all active objects by one timestep.
            for i = 1:length(this.object_list)
                this.object_list{i}.update()
            end
        end
        
        function draw_all(this)
            for i = 1:length(this.object_list)
                this.object_list{i}.draw()
            end
        end
        
        function draw_info(this)
            % For starters, include the time
            y_text = this.plot_size(3) + (this.workspace_buffer * (this.plot_size(4) - this.plot_size(3))) / ((1 +this.workspace_buffer) * 4);
            x_text = this.plot_size(1) + (this.workspace_buffer * (this.plot_size(4) - this.plot_size(3))) / ((1 +this.workspace_buffer) * 4);
            time_str = strcat('Time = ', num2str(this.time));
            text(x_text, y_text, time_str)
        end
        
        function resolve_bound_states(this)
            % For this function we will ASSUME that collisions only happen
            % between two objects at a time. The function will break down
            % for collisions of three or more objects. This should not
            % happen in practice though. 
            % We will also assume ball-ball collisions only happen between
            % unbound balls. Otherwise it will throw an error.
            % The general structure of the algorithm is as follows:
            % The entire object list will be run through. 
            % 1) Hand-Ball collisions: If ball already bound, verify
            % parent, and pathgen algorithm, then check if release criteria
            % are met, and release  if necessary. If the ball is not bound,
            % check if catch criteria is met. If catch criteria is met,
            % bind the ball. Otherwise, bounce it with the proper
            % coefficient of restitution. 
            % 2) Ball-Ball collisions will be treated like an elastic
            % collision. 
            % 3) Hand-Hand collisions will throw a fatal error
            % A hitlist will be kept to ensure collisions are not treated
            % twice.
            hitlist = [];
            for i = 1:length(this.object_list)
                if any(ismember(hitlist, i))
                    % If this object was already hit, continue to next loop
                    % iteration
                    continue
                end
                bb_1 = this.object_list{i}.bounding_box;
                for j = 1:length(this.object_list)
                    if (any(ismember(hitlist, j)) || (i == j))
                        % If this second object was already hit, continue
                        % to next inner loop iteration.
                        continue
                    end
                    bb_2 = this.object_list{j}.bounding_box;
                    if is_intersect(bb_1, bb_2)
                        % They intersect!
                        if hand_ball_interaction(this.object_list{i}, this.object_list{j})
                            % Handle a hand-ball interaction
                            % Determine which is the hand and which is the
                            % ball
                            if strcmp(this.object_list{i}.shape, 'rectangle')
                                h_id = i;
                                b_id = j;
                            else
                                h_id = j;
                                b_id = i;
                            end
                            if this.object_list{b_id}.bound_state
                                % Ball is already bound. Check for release
                                % condition (Hand is traveling at -g +/-
                                % tolerance
                                if (this.object_list{h_id}.ay < (-9.81 - (9.81 * this.throw_tolerance)))
                                    % Release condition is met! Release!
                                    this.object_list{b_id}.bound_state = false;
                                else
                                    % Release condition not met. Ensure it
                                    % is bound to the right parent, and
                                    % pathgen algorithm.
                                    this.object_list{b_id}.parent_id = h_id;
                                    this.object_list{b_id}.bound_state_function = this.object_list{h_id}.bound_state_function;
                                end
                            else
                                % Ball is not bound yet. Check if catch
                                % condition is met.
                                if catch_condition_met(this.object_list{h_id}, this.object_list{b_id}, this.catch_tolerance)
                                    %Catch Condition is met. Bind it.
                                    this.object_list{b_id}.bound_state = true;
                                    this.object_list{b_id}.parent_id = h_id;
                                    this.object_list{b_id}.bound_state_function = this.object_list{h_id}.bound_state_function;
                                else
                                    % The catch condition is not met.
                                    % Bounce the ball.
                                    % Model the hand as infinite mass
                                    % Ignore changes in x
                                    this.object_list{b_id}.vy = this.ball_restitution_coeff * (-this.object_list{b_id}.vy + 2 * this.object_list{h_id}.vy);
                                end 
                            end
                        elseif ball_ball_interaction(this.object_list{i}, this.object_list{j})
                            % Handle a ball_ball interaction
                            error('Ball-Ball collision. Aborting now.')
                        elseif hand_hand_interaction(this.object_list{i}, this.object_list{j})
                            % Handle a hand_hand interaction
                            error('Hand-Hand Collision was detected. Aborting now.')
                        end
                        % Update Hitlist
                        hitlist(end + 1) = i;
                        hitlist(end + 1) = j;
                    end
                end
            end
        end

        function introduce_objects(this)
            kill_list = [];
            for i = 1:length(this.object_hold)
                if this.object_hold{i}.time <= this.time
                    this.object_list{this.object_hold{i}.id_number} = this.object_hold{i};
                    kill_list(end+1) = i;
                end
            end
            for j = 1:length(kill_list)
                this.object_hold(kill_list(j)) = [];
            end
        end
        
        function plot_workspace(this)
            figure(1)
            for i = 1:length(this.workspace_list)
                    min_x = this.workspace_list{i}(1);
                    max_x = this.workspace_list{i}(2);
                    min_y = this.workspace_list{i}(3);
                    max_y = this.workspace_list{i}(4);
                    width = max_x - min_x;
                    height = max_y - min_y;
                rectangle('Position', [min_x, min_y, width, height], 'FaceColor', this.workspace_color);
            end
        end
        
        function set_plot_size(this)
            % Find workspace extremes across all workspaces in list
            min_x = [];
            max_x = [];
            min_y = [];
            max_y = [];
            for i = 1:length(this.workspace_list)
                if i == 1
                    min_x = this.workspace_list{i}(1);
                    max_x = this.workspace_list{i}(2);
                    min_y = this.workspace_list{i}(3);
                    max_y = this.workspace_list{i}(4);
                else
                    if min_x > this.workspace_list{i}(1)
                        min_x = this.workspace_list{i}(1);
                    end
                    if max_x < this.workspace_list{i}(2)
                        max_x = this.workspace_list{i}(2);
                    end
                    if min_y > this.workspace_list{i}(3)
                        min_y = this.workspace_list{i}(3);
                    end
                    if max_y < this.workspace_list{i}(4)
                        max_y = this.workspace_list{i}(4);
                    end
                end
            end
            % Add a buffer to edges to make it more visually pleasing
            x_buffer = this.workspace_buffer * (max_x - min_x);
            max_x = max_x + (x_buffer / 2);
            min_x = min_x - (x_buffer / 2);
            
            y_buffer = this.workspace_buffer * (max_y - min_y);
            max_y = max_y + (y_buffer / 2);
            min_y = min_y - (y_buffer / 2);
            
            % If not square, make plot size square (to avoid distortion)
            % Find aspect ratio. 
            aspect_ratio = (max_x - min_x) / (max_y - min_y);
            if aspect_ratio > 1
                diff = (max_x - min_x) - (max_y - min_y);
                max_y = max_y + (diff / 2);
                min_y = min_y - (diff / 2);
            elseif aspect_ratio < 1
                diff = (max_y - min_y) - (max_x - min_x);
                max_x = max_x + (diff / 2);
                min_x = min_x - (diff / 2);
            end
            this.plot_size = [min_x max_x min_y max_y];
        end
end
end

% Helper Functions
function intersect_bool = is_intersect(bb_1, bb_2)
% Helper function that determines if two bounding boxes are intersecting
min_x1 = bb_1(1);
max_x1 = bb_1(2);
min_y1 = bb_1(3);
max_y1 = bb_1(4);

min_x2 = bb_2(1);
max_x2 = bb_2(2);
min_y2 = bb_2(3);
max_y2 = bb_2(4);

% Return false if left, right, above, or below
if ((min_x1 > max_x2) || (max_x1 < min_x2) || (min_y1 > max_y2) || (max_y1 < min_y2))
    intersect_bool = false;
else
    intersect_bool = true;
end
end

function condition_bool = catch_condition_met(hand_obj, ball_obj, tolerance)
if ((ball_obj.vx >= hand_obj.vx - tolerance * abs(hand_obj.vx)) ...
        && (ball_obj.vx <= hand_obj.vx + tolerance * abs(hand_obj.vx)) ...
        && (ball_obj.vy >= hand_obj.vy - tolerance * abs(hand_obj.vy)) ...
        && (ball_obj.vy <= hand_obj.vy + tolerance * abs(hand_obj.vy)))
    condition_bool = true;
else
    condition_bool = false;
end
end

function interaction_bool = hand_ball_interaction(obj_1, obj_2)
if ((strcmp(obj_1.shape, 'rectangle') && strcmp(obj_2.shape, 'circle')) ...
        || (strcmp(obj_2.shape, 'rectangle') && strcmp(obj_1.shape, 'circle')))
    interaction_bool = true;
else
    interaction_bool = false;
end
end

function interaction_bool = ball_ball_interaction(obj_1, obj_2)
if (strcmp(obj_1.shape, 'circle') && strcmp(obj_2.shape, 'circle'))
    interaction_bool = true;
else
    interaction_bool = false;
end
end

function interaction_bool = hand_hand_interaction(obj_1, obj_2)
if (strcmp(obj_1.shape, 'rectangle') && strcmp(obj_2.shape, 'rectangle'))
    interaction_bool = true;
else
    interaction_bool = false;
end
end
