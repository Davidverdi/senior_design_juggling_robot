classdef pp_data_class < handle
    % pp_data_class.m
    % This is the path planning data class which is a data structure for
    % organizing and and holding the path planning data.
    % David Verdi
    % Created: 1/31/2017
    % Updated: 2/2/2017
    
    % USER GUIDE FOR THIS CLASS:
    % NOTE: "this" isn't an acutal function argument. It signifies that the
    % funciton is a member function for a class. 
    % i.e. pp_data_object.set_workspace(bbox)
    %
    %
    % To set the workpaces and cycles, use the functions:
    % set_workspace1(this, bbox)
    % set_workspace2(this, bbox)
    % set_cycles(this, num_cycles)
    %
    % To add rampup, loop, and rampdown polynomials, use functions:
    % add_rampup(this, hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
    % add_loop(this, hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
    % add_rampdown(this, hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
    %
    % To access the data in this class, use the getter methods:
    % cycles = get_cycles(this)
    %
    % To get the number of periods in the rampup or rampdown:
    % get_rampup_periods(this, hand, direction)
    % get_loop_periods(this, hand, direction)
    % get_rampdown_periods(this, hand, direction)
    % 
    % To get the duration of a specific period:
    % get_rampup_durations(this, hand, direction)
    % get_loop_durations(this, hand, direction)
    % get_rampdown_durations(this, hand, direction)
    % 
    % To get a specific polynomial
    % get_rampup_dpoly(this, hand, direction, period)
    % get_rampup_vpoly(this, hand, direction, period)
    % get_rampup_apoly(this, hand, direction, period)
    % get_rampup_jpoly(this, hand, direction, period)
    % get_loop_dpoly(this, hand, direction, period)
    % get_loop_vpoly(this, hand, direction, period)
    % get_loop_apoly(this, hand, direction, period)
    % get_loop_jpoly(this, hand, direction, period)
    % get_rampdown_dpoly(this, hand, direction, period)
    % get_rampdown_vpoly(this, hand, direction, period)
    % get_rampdown_apoly(this, hand, direction, period)
    % get_rampdown_jpoly(this, hand, direction, period)
    
    
    
    
    
    properties
        % General Properties
        cycles
        workspace1 %[min_x max_x min_y max_y]
        workspace2 %[min_x max_x min_y max_y]
        
        % Number of periods in the piecewise function (For x and y)
        % These are scalars
        hand1_x_rampup_periods
        hand1_y_rampup_periods
        hand1_x_loop_periods
        hand1_y_loop_periods
        hand1_x_rampdown_periods
        hand1_y_rampdown_periods
        
        hand2_x_rampup_periods
        hand2_y_rampup_periods
        hand2_x_loop_periods
        hand2_y_loop_periods
        hand2_x_rampdown_periods
        hand2_y_rampdown_periods
        
        % Durations in the periods
        % These are 1xn vectors, where n is the number of periods.
        hand1_x_rampup_durations
        hand1_y_rampup_durations
        hand1_x_loop_durations
        hand1_y_loop_durations
        hand1_x_rampdown_durations
        hand1_y_rampdown_durations
        
        hand2_x_rampup_durations
        hand2_y_rampup_durations
        hand2_x_loop_durations
        hand2_y_loop_durations
        hand2_x_rampdown_durations
        hand2_y_rampdown_durations
        
        % Polynomials
        % These are 4xn cell arrays, where n is the number of periods
        % First row is distance, second row is velocity, third row is
        % acceleration, fourth row is jerk. 
        hand1_x_rampup_polynomials
        hand1_y_rampup_polynomials
        hand1_x_loop_polynomials
        hand1_y_loop_polynomials
        hand1_x_rampdown_polynomials
        hand1_y_rampdown_polynomials
        
        hand2_x_rampup_polynomials
        hand2_y_rampup_polynomials
        hand2_x_loop_polynomials
        hand2_y_loop_polynomials
        hand2_x_rampdown_polynomials
        hand2_y_rampdown_polynomials
    end
    
    methods
        % Constructor
        function this = pp_data_class()
            this.cycles = 0;
            this.workspace1 = [];
            this.workspace2 = [];
            
            this.hand1_x_rampup_periods = 0;
            this.hand1_y_rampup_periods = 0;
            this.hand1_x_loop_periods = 0;
            this.hand1_y_loop_periods = 0;
            this.hand1_x_rampdown_periods = 0;
            this.hand1_y_rampdown_periods = 0;

            this.hand2_x_rampup_periods = 0;
            this.hand2_y_rampup_periods = 0;
            this.hand2_x_loop_periods = 0;
            this.hand2_y_loop_periods = 0;
            this.hand2_x_rampdown_periods = 0;
            this.hand2_y_rampdown_periods = 0;
            
            this.hand1_x_rampup_durations = [];
            this.hand1_y_rampup_durations = [];
            this.hand1_x_loop_durations = [];
            this.hand1_y_loop_durations = [];
            this.hand1_x_rampdown_durations = [];
            this.hand1_y_rampdown_durations = [];

            this.hand2_x_rampup_durations = [];
            this.hand2_y_rampup_durations = [];
            this.hand2_x_loop_durations = [];
            this.hand2_y_loop_durations = [];
            this.hand2_x_rampdown_durations = [];
            this.hand2_y_rampdown_durations = [];
            
            this.hand1_x_rampup_polynomials = {};
            this.hand1_y_rampup_polynomials = {};
            this.hand1_x_loop_polynomials = {};
            this.hand1_y_loop_polynomials = {};
            this.hand1_x_rampdown_polynomials = {};
            this.hand1_y_rampdown_polynomials = {};

            this.hand2_x_rampup_polynomials = {};
            this.hand2_y_rampup_polynomials = {};
            this.hand2_x_loop_polynomials = {};
            this.hand2_y_loop_polynomials = {};
            this.hand2_x_rampdown_polynomials = {};
            this.hand2_y_rampdown_polynomials = {};
        end
        
        % Setters
        function set_workspace1(this, bbox)
            % Sets workspace1
            %bbox = [min_x max_x min_y max_y]
            this.workspace1 = bbox;
        end
        
        function set_workspace2(this, bbox)
            % Sets workspace2
            % bbox = [min_x max_x min_y max_y]
            this.workspace2 = bbox;
        end
        
        function set_cycles(this, num_cycles)
            this.cycles = num_cycles;
        end
        
        function add_rampup(this, hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
            % Adds a rampup segment to the piece-wise rampup function. 
            % Hand number is 1 or 2
            % Direction is x or y
            % Duration is the time that the pieciewise segment lasts
            % d_poly is the distance polynomial
            % v_poly is the velocity polynomial
            % a_poly is the acceleration polynomial
            % j_poly is the jerk polynomial.
            
            % Branch off by direction, then by hand number. Set all the
            % variables accordingly
            if is_x(direction)
                % We are operating on x values
                if hand_number == 1
                    % We are operating on x values of hand1
                    % Increment the period count
                    this.hand1_x_rampup_periods = this.hand1_x_rampup_periods + 1;
                    % append a duration to the duration list. 
                    this.hand1_x_rampup_durations(this.hand1_x_rampup_periods) = duration;
                    % Add the polynomials
                    this.hand1_x_rampup_polynomials{1, this.hand1_x_rampup_periods} = d_poly;
                    this.hand1_x_rampup_polynomials{2, this.hand1_x_rampup_periods} = v_poly;
                    this.hand1_x_rampup_polynomials{3, this.hand1_x_rampup_periods} = a_poly;
                    this.hand1_x_rampup_polynomials{4, this.hand1_x_rampup_periods} = j_poly;
                elseif hand_number == 2
                    % we are operating on x values of hand2
                    % We are operating on x values of hand2
                    % Increment the period count
                    this.hand2_x_rampup_periods = this.hand2_x_rampup_periods + 1;
                    % append a duration to the duration list. 
                    this.hand2_x_rampup_durations(this.hand2_x_rampup_periods) = duration;
                    % Add the polynomials
                    this.hand2_x_rampup_polynomials{1, this.hand2_x_rampup_periods} = d_poly;
                    this.hand2_x_rampup_polynomials{2, this.hand2_x_rampup_periods} = v_poly;
                    this.hand2_x_rampup_polynomials{3, this.hand2_x_rampup_periods} = a_poly;
                    this.hand2_x_rampup_polynomials{4, this.hand2_x_rampup_periods} = j_poly;
                else
                    error('Hand number is unrecognized');
                end
            elseif is_y(direction)
                % We are operating on y values
                if hand_number == 1
                    % We are operating on y values of hand1
                    % Increment the period count
                    this.hand1_y_rampup_periods = this.hand1_y_rampup_periods + 1;
                    % append a duration to the duration list. 
                    this.hand1_y_rampup_durations(this.hand1_y_rampup_periods) = duration;
                    % Add the polynomials
                    this.hand1_y_rampup_polynomials{1, this.hand1_y_rampup_periods} = d_poly;
                    this.hand1_y_rampup_polynomials{2, this.hand1_y_rampup_periods} = v_poly;
                    this.hand1_y_rampup_polynomials{3, this.hand1_y_rampup_periods} = a_poly;
                    this.hand1_y_rampup_polynomials{4, this.hand1_y_rampup_periods} = j_poly;
                elseif hand_number == 2
                    % we are operating on y values of hand2
                    % We are operating on y values of hand2
                    % Increment the period count
                    this.hand2_y_rampup_periods = this.hand2_y_rampup_periods + 1;
                    % append a duration to the duration list. 
                    this.hand2_y_rampup_durations(this.hand2_y_rampup_periods) = duration;
                    % Add the polynomials
                    this.hand2_y_rampup_polynomials{1, this.hand2_y_rampup_periods} = d_poly;
                    this.hand2_y_rampup_polynomials{2, this.hand2_y_rampup_periods} = v_poly;
                    this.hand2_y_rampup_polynomials{3, this.hand2_y_rampup_periods} = a_poly;
                    this.hand2_y_rampup_polynomials{4, this.hand2_y_rampup_periods} = j_poly;
                else
                    error('Hand number is unrecognized');
                end
            else
                error('Direction is unrecognized');
            end
        end
        
        function add_loop(this, hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
            % Adds a loop segment to the piece-wise loop function. 
            % Hand number is 1 or 2
            % Direction is x or y
            % Duration is the time that the pieciewise segment lasts
            % d_poly is the distance polynomial
            % v_poly is the velocity polynomial
            % a_poly is the acceleration polynomial
            % j_poly is the jerk polynomial.
            
            % Branch off by direction, then by hand number. Set all the
            % variables accordingly
            if is_x(direction)
                % We are operating on x values
                if hand_number == 1
                    % We are operating on x values of hand1
                    % Increment the period count
                    this.hand1_x_loop_periods = this.hand1_x_loop_periods + 1;
                    % append a duration to the duration list. 
                    this.hand1_x_loop_durations(this.hand1_x_loop_periods) = duration;
                    % Add the polynomials
                    this.hand1_x_loop_polynomials{1, this.hand1_x_loop_periods} = d_poly;
                    this.hand1_x_loop_polynomials{2, this.hand1_x_loop_periods} = v_poly;
                    this.hand1_x_loop_polynomials{3, this.hand1_x_loop_periods} = a_poly;
                    this.hand1_x_loop_polynomials{4, this.hand1_x_loop_periods} = j_poly;
                elseif hand_number == 2
                    % we are operating on x values of hand2
                    % We are operating on x values of hand2
                    % Increment the period count
                    this.hand2_x_loop_periods = this.hand2_x_loop_periods + 1;
                    % append a duration to the duration list. 
                    this.hand2_x_loop_durations(this.hand2_x_loop_periods) = duration;
                    % Add the polynomials
                    this.hand2_x_loop_polynomials{1, this.hand2_x_loop_periods} = d_poly;
                    this.hand2_x_loop_polynomials{2, this.hand2_x_loop_periods} = v_poly;
                    this.hand2_x_loop_polynomials{3, this.hand2_x_loop_periods} = a_poly;
                    this.hand2_x_loop_polynomials{4, this.hand2_x_loop_periods} = j_poly;
                else
                    error('Hand number is unrecognized');
                end
            elseif is_y(direction)
                % We are operating on y values
                if hand_number == 1
                    % We are operating on y values of hand1
                    % Increment the period count
                    this.hand1_y_loop_periods = this.hand1_y_loop_periods + 1;
                    % append a duration to the duration list. 
                    this.hand1_y_loop_durations(this.hand1_y_loop_periods) = duration;
                    % Add the polynomials
                    this.hand1_y_loop_polynomials{1, this.hand1_y_loop_periods} = d_poly;
                    this.hand1_y_loop_polynomials{2, this.hand1_y_loop_periods} = v_poly;
                    this.hand1_y_loop_polynomials{3, this.hand1_y_loop_periods} = a_poly;
                    this.hand1_y_loop_polynomials{4, this.hand1_y_loop_periods} = j_poly;
                elseif hand_number == 2
                    % we are operating on y values of hand2
                    % We are operating on y values of hand2
                    % Increment the period count
                    this.hand2_y_loop_periods = this.hand2_y_loop_periods + 1;
                    % append a duration to the duration list. 
                    this.hand2_y_loop_durations(this.hand2_y_loop_periods) = duration;
                    % Add the polynomials
                    this.hand2_y_loop_polynomials{1, this.hand2_y_loop_periods} = d_poly;
                    this.hand2_y_loop_polynomials{2, this.hand2_y_loop_periods} = v_poly;
                    this.hand2_y_loop_polynomials{3, this.hand2_y_loop_periods} = a_poly;
                    this.hand2_y_loop_polynomials{4, this.hand2_y_loop_periods} = j_poly;
                else
                    error('Hand number is unrecognized');
                end
            else
                error('Direction is unrecognized');
            end
        end
        
        function add_rampdown(this, hand_number, direction, duration, d_poly, v_poly, a_poly, j_poly)
        % Adds a rampdown segment to the piece-wise rampdown function. 
            % Hand number is 1 or 2
            % Direction is x or y
            % Duration is the time that the pieciewise segment lasts
            % d_poly is the distance polynomial
            % v_poly is the velocity polynomial
            % a_poly is the acceleration polynomial
            % j_poly is the jerk polynomial.
            
            % Branch off by direction, then by hand number. Set all the
            % variables accordingly
            if is_x(direction)
                % We are operating on x values
                if hand_number == 1
                    % We are operating on x values of hand1
                    % Increment the period count
                    this.hand1_x_rampdown_periods = this.hand1_x_rampdown_periods + 1;
                    % append a duration to the duration list. 
                    this.hand1_x_rampdown_durations(this.hand1_x_rampdown_periods) = duration;
                    % Add the polynomials
                    this.hand1_x_rampdown_polynomials{1, this.hand1_x_rampdown_periods} = d_poly;
                    this.hand1_x_rampdown_polynomials{2, this.hand1_x_rampdown_periods} = v_poly;
                    this.hand1_x_rampdown_polynomials{3, this.hand1_x_rampdown_periods} = a_poly;
                    this.hand1_x_rampdown_polynomials{4, this.hand1_x_rampdown_periods} = j_poly;
                elseif hand_number == 2
                    % we are operating on x values of hand2
                    % We are operating on x values of hand2
                    % Increment the period count
                    this.hand2_x_rampdown_periods = this.hand2_x_rampdown_periods + 1;
                    % append a duration to the duration list. 
                    this.hand2_x_rampdown_durations(this.hand2_x_rampdown_periods) = duration;
                    % Add the polynomials
                    this.hand2_x_rampdown_polynomials{1, this.hand2_x_rampdown_periods} = d_poly;
                    this.hand2_x_rampdown_polynomials{2, this.hand2_x_rampdown_periods} = v_poly;
                    this.hand2_x_rampdown_polynomials{3, this.hand2_x_rampdown_periods} = a_poly;
                    this.hand2_x_rampdown_polynomials{4, this.hand2_x_rampdown_periods} = j_poly;
                else
                    error('Hand number is unrecognized');
                end
            elseif is_y(direction)
                % We are operating on y values
                if hand_number == 1
                    % We are operating on y values of hand1
                    % Increment the period count
                    this.hand1_y_rampdown_periods = this.hand1_y_rampdown_periods + 1;
                    % append a duration to the duration list. 
                    this.hand1_y_rampdown_durations(this.hand1_y_rampdown_periods) = duration;
                    % Add the polynomials
                    this.hand1_y_rampdown_polynomials{1, this.hand1_y_rampdown_periods} = d_poly;
                    this.hand1_y_rampdown_polynomials{2, this.hand1_y_rampdown_periods} = v_poly;
                    this.hand1_y_rampdown_polynomials{3, this.hand1_y_rampdown_periods} = a_poly;
                    this.hand1_y_rampdown_polynomials{4, this.hand1_y_rampdown_periods} = j_poly;
                elseif hand_number == 2
                    % we are operating on y values of hand2
                    % We are operating on y values of hand2
                    % Increment the period count
                    this.hand2_y_rampdown_periods = this.hand2_y_rampdown_periods + 1;
                    % append a duration to the duration list. 
                    this.hand2_y_rampdown_durations(this.hand2_y_rampdown_periods) = duration;
                    % Add the polynomials
                    this.hand2_y_rampdown_polynomials{1, this.hand2_y_rampdown_periods} = d_poly;
                    this.hand2_y_rampdown_polynomials{2, this.hand2_y_rampdown_periods} = v_poly;
                    this.hand2_y_rampdown_polynomials{3, this.hand2_y_rampdown_periods} = a_poly;
                    this.hand2_y_rampdown_polynomials{4, this.hand2_y_rampdown_periods} = j_poly;
                else
                    error('Hand number is unrecognized');
                end
            else
                error('Direction is unrecognized');
            end
        end
        
        % Getters
        function cycles = get_cycles(this)
            cycles = this.cycles;
        end
        
        % Period getters
        % Rampup period getter
        function periods = get_rampup_periods(this, hand, direction)
            if hand == 1
                if is_x(direction)
                    periods = this.hand1_x_rampup_periods;
                elseif is_y(direction)
                    periods = this.hand1_y_rampup_periods;
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    periods = this.hand2_x_rampup_periods;
                elseif is_y(direction)
                    periods = this.hand2_y_rampup_periods;
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        %Loop period Getter
        function periods = get_loop_periods(this, hand, direction)
            if hand == 1
                if is_x(direction)
                    periods = this.hand1_x_loop_periods;
                elseif is_y(direction)
                    periods = this.hand1_y_loop_periods;
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    periods = this.hand2_x_loop_periods;
                elseif is_y(direction)
                    periods = this.hand2_y_loop_periods;
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function periods = get_rampdown_periods(this, hand, direction)
            if hand == 1
                if is_x(direction)
                    periods = this.hand1_x_rampdown_periods;
                elseif is_y(direction)
                    periods = this.hand1_y_rampdown_periods;
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    periods = this.hand2_x_rampdown_periods;
                elseif is_y(direction)
                    periods = this.hand2_y_rampdown_periods;
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        % Duration getters
        % Rampup duration getter
        function durations = get_rampup_durations(this, hand, direction)
            if hand == 1
                if is_x(direction)
                    durations = this.hand1_x_rampup_durations;
                elseif is_y(direction)
                    durations = this.hand1_y_rampup_durations;
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    durations = this.hand2_x_rampup_durations;
                elseif is_y(direction)
                    durations = this.hand2_y_rampup_durations;
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        % loop duration getter
        function durations = get_loop_durations(this, hand, direction)
            if hand == 1
                if is_x(direction)
                    durations = this.hand1_x_loop_durations;
                elseif is_y(direction)
                    durations = this.hand1_y_loop_durations;
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    durations = this.hand2_x_loop_durations;
                elseif is_y(direction)
                    durations = this.hand2_y_loop_durations;
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        % rampdown duration getter
        function durations = get_rampdown_durations(this, hand, direction)
            if hand == 1
                if is_x(direction)
                    durations = this.hand1_x_rampdown_durations;
                elseif is_y(direction)
                    durations = this.hand1_y_rampdown_durations;
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    durations = this.hand2_x_rampdown_durations;
                elseif is_y(direction)
                    durations = this.hand2_y_rampdown_durations;
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        % Polynomial getters
        % Rampup Polynomial Getters
        function d_poly = get_rampup_dpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    d_poly = this.hand1_x_rampup_polynomials{1, period};
                elseif is_y(direction)
                    d_poly = this.hand1_y_rampup_polynomials{1, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    d_poly = this.hand2_x_rampup_polynomials{1, period};
                elseif is_y(direction)
                    d_poly = this.hand2_y_rampup_polynomials{1, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function v_poly = get_rampup_vpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    v_poly = this.hand1_x_rampup_polynomials{2, period};
                elseif is_y(direction)
                    v_poly = this.hand1_y_rampup_polynomials{2, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    v_poly = this.hand2_x_rampup_polynomials{2, period};
                elseif is_y(direction)
                    v_poly = this.hand2_y_rampup_polynomials{2, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function a_poly = get_rampup_apoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    a_poly = this.hand1_x_rampup_polynomials{3, period};
                elseif is_y(direction)
                    a_poly = this.hand1_y_rampup_polynomials{3, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    a_poly = this.hand2_x_rampup_polynomials{3, period};
                elseif is_y(direction)
                    a_poly = this.hand2_y_rampup_polynomials{3, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function j_poly = get_rampup_jpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    j_poly = this.hand1_x_rampup_polynomials{4, period};
                elseif is_y(direction)
                    j_poly = this.hand1_y_rampup_polynomials{4, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    j_poly = this.hand2_x_rampup_polynomials{4, period};
                elseif is_y(direction)
                    j_poly = this.hand2_y_rampup_polynomials{4, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        % Loop Getters
        function d_poly = get_loop_dpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    d_poly = this.hand1_x_loop_polynomials{1, period};
                elseif is_y(direction)
                    d_poly = this.hand1_y_loop_polynomials{1, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    d_poly = this.hand2_x_loop_polynomials{1, period};
                elseif is_y(direction)
                    d_poly = this.hand2_y_loop_polynomials{1, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function v_poly = get_loop_vpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    v_poly = this.hand1_x_loop_polynomials{2, period};
                elseif is_y(direction)
                    v_poly = this.hand1_y_loop_polynomials{2, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    v_poly = this.hand2_x_loop_polynomials{2, period};
                elseif is_y(direction)
                    v_poly = this.hand2_y_loop_polynomials{2, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function a_poly = get_loop_apoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    a_poly = this.hand1_x_loop_polynomials{3, period};
                elseif is_y(direction)
                    a_poly = this.hand1_y_loop_polynomials{3, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    a_poly = this.hand2_x_loop_polynomials{3, period};
                elseif is_y(direction)
                    a_poly = this.hand2_y_loop_polynomials{3, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function j_poly = get_loop_jpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    j_poly = this.hand1_x_loop_polynomials{4, period};
                elseif is_y(direction)
                    j_poly = this.hand1_y_loop_polynomials{4, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    j_poly = this.hand2_x_loop_polynomials{4, period};
                elseif is_y(direction)
                    j_poly = this.hand2_y_loop_polynomials{4, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        % Rampdown Getters
        function d_poly = get_rampdown_dpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    d_poly = this.hand1_x_rampdown_polynomials{1, period};
                elseif is_y(direction)
                    d_poly = this.hand1_y_rampdown_polynomials{1, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    d_poly = this.hand2_x_rampdown_polynomials{1, period};
                elseif is_y(direction)
                    d_poly = this.hand2_y_rampdown_polynomials{1, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function v_poly = get_rampdown_vpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    v_poly = this.hand1_x_rampdown_polynomials{2, period};
                elseif is_y(direction)
                    v_poly = this.hand1_y_rampdown_polynomials{2, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    v_poly = this.hand2_x_rampdown_polynomials{2, period};
                elseif is_y(direction)
                    v_poly = this.hand2_y_rampdown_polynomials{2, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function a_poly = get_rampdown_apoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    a_poly = this.hand1_x_rampdown_polynomials{3, period};
                elseif is_y(direction)
                    a_poly = this.hand1_y_rampdown_polynomials{3, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    a_poly = this.hand2_x_rampdown_polynomials{3, period};
                elseif is_y(direction)
                    a_poly = this.hand2_y_rampdown_polynomials{3, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end
        
        function j_poly = get_rampdown_jpoly(this, hand, direction, period)
            if hand == 1
                if is_x(direction)
                    j_poly = this.hand1_x_rampdown_polynomials{4, period};
                elseif is_y(direction)
                    j_poly = this.hand1_y_rampdown_polynomials{4, period};
                else
                    error('Unrecognized direction')
                end
            elseif hand == 2
                if is_x(direction)
                    j_poly = this.hand2_x_rampdown_polynomials{4, period};
                elseif is_y(direction)
                    j_poly = this.hand2_y_rampdown_polynomials{4, period};
                else
                    error('Unrecognized direction')
                end
            else
                error('Unrecognized hand')
            end
        end     
    end
end

% Helper Functions
function truth_value = is_x(direction)
if (strcmp(direction, 'x') || strcmp(direction, 'X'))
    truth_value = true;
else
    truth_value = false;
end
end

function truth_value = is_y(direction)
if (strcmp(direction, 'y') || strcmp(direction, 'Y'))
    truth_value = true;
else
    truth_value = false;
end
end
