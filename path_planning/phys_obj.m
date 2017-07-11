classdef phys_obj < handle
    % phys_obj.m
    % This is the phys_obj (physical object) class, which will be all
    % physical objects in the simulation. These objects hold positions,
    % velocities, accelerations, bound and unbound equations of motion,
    % etc. 
    %
    % David Verdi
    % Created: 12/7/2016
    % Updated: 1/13/2017
    
    properties
        % Identity Number (starts from 1)
        id_number
        
        % Basic Kinematic Properties
        x
        y
        vx
        vy
        ax
        ay
        jx
        jy
        
        %Time Properties
        time
        timestep
        
        % Bounding Box is used for collision detection. 
        % For a rectangle, it is the outline. For a circle, it is the
        % inscribed square.
        bounding_box % [min_x max_x min_y max_y]
        
        % Reduction factor for ball to make it more like a point particle
        reduction_factor = 0.001
        
        % Geometry
        shape % 'circle' or 'rectangle'
        size % [radius] or [x_size y_size]
        color
        mass
        outline = 'b';
        outline_width = 1;
        
        % Bound_state allows us to switch between planned path motion and 
        % freefall motion 
        bound_state % true or false
        parent_id % identity number of the object that this object is bound to
        bound_state_function % path plan function
        bound_state_data % cell array of path planning polynomials
    end
    
    methods
        % constructor
        function this = phys_obj(id_number, ini_cond, time0, timestep, shape, size, color, bound_state, parent_id, bound_state_function, bound_state_data)
            % this is the constructor.
            if nargin > 0
                this.id_number = id_number;
                % ini_cond is [x y vx vy ax ay jx jy]
                this.x = ini_cond(1);
                this.y = ini_cond(2); 
                this.vx = ini_cond(3);
                this.vy = ini_cond(4); 
                this.ax = ini_cond(5); 
                this.ay = ini_cond(6); 
                this.jx = ini_cond(7);
                this.jy = ini_cond(8);
                this.timestep = timestep;
                this.time = time0;
                this.shape = shape; % 'circle' or 'rectangle'
                this.size = size;
                this.color = color;
                this.bound_state = bound_state; % true or false
                this.bound_state_function = bound_state_function;
                this.bound_state_data = bound_state_data;
                this.parent_id = parent_id;
                this.update_bounding_box();
            end
        end
        
        function update_bounding_box(this)
            box = zeros(1,4);
            if strcmp(this.shape, 'circle')
                % Make the ball like a point particle (1% of volume is
                % bounding box. 
                % Generate bounding box as inscribed square
                % In this case, size is radius
                box(1) = this.x - (1/sqrt(2))*this.reduction_factor*this.size(1); %min_x
                box(2) = this.x + (1/sqrt(2))*this.reduction_factor*this.size(1); %max_x
                box(3) = this.y - (1/sqrt(2))*this.reduction_factor*this.size(1); %min_y
                box(4) = this.y + (1/sqrt(2))*this.reduction_factor*this.size(1); %max_y
            end
            
            if strcmp(this.shape,'rectangle')
                % Generate bounding box as per drawing, with this.x and
                % this.y at the top center edge.
                % In this case, size is [x_size y_size]
                box(1) = this.x - 0.5*this.size(1); %min_x
                box(2) = this.x + 0.5*this.size(1); %max_x
                box(3) = this.y - this.size(2); %min_y
                box(4) = this.y; %max_y
            end
            this.bounding_box = box;
        end
        
        function update(this)
            this.time = this.time + this.timestep;
            this.update_kinematics();
            this.update_bounding_box();
        end
        
        function update_kinematics(this)
            % If the system is in a bound state, it gets the kinematic
            % values from a function file. Otherwise, it uses freefall to 
            % update the kinematics.
            if this.bound_state == true
                [this.x, this.y, this.vx, this.vy, this.ax, this.ay, this.jx, this.jy] = this.bound_state_function(this.bound_state_data, this.time);
                this.outline = 'b';
            end
            if this.bound_state == false
                % vx does not change, since ax = 0
                this.x = this.x + this.vx*this.timestep;
                this.ax = 0;
                this.jx = 0;
                % update acceleration first, so transitions will work
                this.jy = 0;
                this.ay = -9.81; %m/s^2
                this.y = this.y + this.vy*this.timestep + 0.5*this.ay*this.timestep^2;
                this.vy = this.vy + this.ay*this.timestep;
                this.outline = 'k';
            end
        end
        
        function draw(this)
            if strcmp(this.shape, 'circle')
                % Draw circle centered on this.x and this.y
                x_draw = this.x - this.size(1);
                y_draw = this.y - this.size(1);
                rectangle('Position',[x_draw y_draw 2*this.size(1) 2*this.size(1)],'FaceColor',this.color,'Curvature',[1 1], 'EdgeColor', this.outline, 'LineWidth', this.outline_width)
            end
            
            if strcmp(this.shape,'rectangle')
                % Plots a rectangle with this.x and this.y being located at
                % the center of the top edge of the rectangle:
                % ----x----
                % |        |
                % ---------
                % Matlab positions based on the lower right hand corner of
                % the box
                x_draw = this.x - 0.5*this.size(1);
                y_draw = this.y - this.size(2);
                rectangle('Position',[x_draw y_draw this.size(1) this.size(2)],'FaceColor',this.color)
            end
        end
    end
    
end

