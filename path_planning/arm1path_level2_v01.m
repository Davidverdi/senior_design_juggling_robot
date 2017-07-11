function [x, y, vx, vy, ax, ay, jx, jy] = arm1path_level2_v01(pp_data, t)
% arm1path_level2_v01.m
% David Verdi
% Created: 2/8/2017
% Updated: 2/8/2017

% LEVEL TWO OF SUCESS!

% Piecewise acceleration profiling using a three step bang-bang approach on
% throw-catch, and on catch-throw. This version uses the pp_data_class
% object as the data structure to store polynomials.

% For refrence, the getter functions for the data structure are as follows:
% To access the data in this class, use the getter methods:
% cycles = get_cycles(this)
%
% To get the number of periods in the rampup or rampdown:
% get_rampup_periods(this, hand, direction)
% get_loop_periods(this, hand, direction)
% get_rampdown_periods(this, hand, direction)
% 
% To get the duration of a specific period:
% get_rampup_duration(this, hand, direction, period)-> Returns duration vec
% get_loop_duration(this, hand, direction, period)-> Returns duration vec
% get_rampdown_duration(this, hand, direction, period)-> Returns duration vec
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


%% Get Cycles and Durations
cycles = pp_data.get_cycles();
ru_periods_y = pp_data.get_rampup_periods(1,'y');
loop_periods_y = pp_data.get_loop_periods(1, 'y');
rd_periods_y = pp_data.get_rampdown_periods(1,'y');

ru_durations_y = pp_data.get_rampup_durations(1, 'y');
loop_durations_y = pp_data.get_loop_durations(1, 'y');
rd_durations_y = pp_data.get_rampdown_durations(1, 'y');

t_rampup_y = sum(ru_durations_y);
t_loop_y = sum(loop_durations_y);
t_rampdown_y = sum(rd_durations_y);

ru_periods_x = pp_data.get_rampup_periods(1,'x');
loop_periods_x = pp_data.get_loop_periods(1, 'x');
rd_periods_x = pp_data.get_rampdown_periods(1,'x');

ru_durations_x = pp_data.get_rampup_durations(1, 'x');
loop_durations_x = pp_data.get_loop_durations(1, 'x');
rd_durations_x = pp_data.get_rampdown_durations(1, 'x');

t_rampup_x= sum(ru_durations_x);
t_loop_x = sum(loop_durations_x);
t_rampdown_x = sum(rd_durations_x);



%% Set X motion
% Motion is in y direction only.
% Determine if desired time is during ramp-up
if t <= t_rampup_x
    % We are in Ramp-up Territory.
    % Loop through all the periods in the ramp-up sequence:
    for i = 1:ru_periods_x
        if t <= sum(ru_durations_x(1:i))
            local_time = t;
            if i > 1
                % On the first period, time starts at zero anyway, and
                % there is a one to one correlation between local time and
                % t in the first period of rampup. 
                local_time = local_time - sum(ru_durations_x(1:(i-1)));
            end
            x = polyval(pp_data.get_rampup_dpoly(1, 'x', i), local_time);
            vx = polyval(pp_data.get_rampup_vpoly(1, 'x', i), local_time);
            ax = polyval(pp_data.get_rampup_apoly(1, 'x', i), local_time);
            jx = polyval(pp_data.get_rampup_jpoly(1, 'x', i), local_time);
            break
        end
    end
    % Determine if polynomial is in rampdown territory
elseif t > (t_rampup_x + (cycles * t_loop_x))
    % We are in rampdown territory. 
    % Loop through all rampdown periods.
    for i = 1:rd_periods_x
        if t <= (t_rampup_x + (cycles * t_loop_x) + sum(rd_durations_x(1:i)))
            % Ramp-down i Territory
            local_time = t - (t_rampup_x + (cycles * t_loop_x)); 
            if i > 1
                % On the first period, time starts at zero anyway, and
                % there is a one to one correlation between local time and
                % t in the first period of rampup. 
                local_time = local_time - sum(rd_durations_y(1:(i-1)));
            end
            x = polyval(pp_data.get_rampdown_dpoly(1, 'x', i), local_time);
            vx = polyval(pp_data.get_rampdown_vpoly(1, 'x', i), local_time);
            ax = polyval(pp_data.get_rampdown_apoly(1, 'x', i), local_time);
            jx = polyval(pp_data.get_rampdown_jpoly(1, 'x', i), local_time);
            % The return ensures that the loop will stop at the first
            % possible time that is less than a period duration. (Turns and
            % if into an elseif
            break
        end
    end
    if t > (t_rampup_x + (cycles * t_loop_x) + t_rampdown_x)
        % Simulation is over
        x = 0;
        vx = 0;
        ax = 0;
        jx = 0;
    end
else
    % We are somewhere in the periodic part of the function.
    % Set a loop time. t_loop_x = 0 means the beginning of the loop.
    % t_period is the end of the loop and is equivalent to 0
    t_in_loop = (rem((t - t_rampup_x), t_loop_x));
    for i = 1:loop_periods_x
        if t_in_loop <= sum(loop_durations_y(1:i))
            local_time = t_in_loop;
            if i > 1
                % On the first period, time starts at zero anyway, and
                % there is a one to one correlation between local time and
                % t in the first period of rampup. 
                local_time = local_time - sum(loop_durations_y(1:(i-1)));
            end
            x = polyval(pp_data.get_loop_dpoly(1, 'x', i), local_time);
            vx = polyval(pp_data.get_loop_vpoly(1, 'x', i), local_time);
            ax = polyval(pp_data.get_loop_apoly(1, 'x', i), local_time);
            jx = polyval(pp_data.get_loop_jpoly(1, 'x', i), local_time);
            break
        end
    end
end

%% Set Y Motion
% Determine if desired time is during ramp-up
if t <= t_rampup_y
    % We are in Ramp-up Territory.
    % Loop through all the periods in the ramp-up sequence:
    for i = 1:ru_periods_y
        if t <= sum(ru_durations_y(1:i))
            local_time = t;
            if i > 1
                % On the first period, time starts at zero anyway, and
                % there is a one to one correlation between local time and
                % t in the first period of rampup. 
                local_time = local_time - sum(ru_durations_y(1:(i-1)));
            end
            y = polyval(pp_data.get_rampup_dpoly(1, 'y', i), local_time);
            vy = polyval(pp_data.get_rampup_vpoly(1, 'y', i), local_time);
            ay = polyval(pp_data.get_rampup_apoly(1, 'y', i), local_time);
            jy = polyval(pp_data.get_rampup_jpoly(1, 'y', i), local_time);
            break
        end
    end
    % Determine if polynomial is in rampdown territory
elseif t > (t_rampup_y + (cycles * t_loop_y))
    % We are in rampdown territory. 
    % Loop through all rampdown periods.
    for i = 1:rd_periods_y
        if t <= (t_rampup_y + (cycles * t_loop_y) + sum(rd_durations_y(1:i)))
            % Ramp-down i Territory
            local_time = t - (t_rampup_y + (cycles * t_loop_y)); 
            if i > 1
                % On the first period, time starts at zero anyway, and
                % there is a one to one correlation between local time and
                % t in the first period of rampup. 
                local_time = local_time - sum(rd_durations_y(1:(i-1)));
            end
            y = polyval(pp_data.get_rampdown_dpoly(1, 'y', i), local_time);
            vy = polyval(pp_data.get_rampdown_vpoly(1, 'y', i), local_time);
            ay = polyval(pp_data.get_rampdown_apoly(1, 'y', i), local_time);
            jy = polyval(pp_data.get_rampdown_jpoly(1, 'y', i), local_time);
            % The return ensures that the loop will stop at the first
            % possible time that is less than a period duration. (Turns and
            % if into an elseif
            break
        end
    end
    if t > (t_rampup_y + (cycles * t_loop_y) + t_rampdown_y)
        % Simulation is over
        y = 0;
        vy = 0;
        ay = 0;
        jy = 0;
    end
else
    % We are somewhere in the periodic part of the function.
    % Set a loop time. t_loop_y = 0 means the beginning of the loop.
    % t_period is the end of the loop and is equivalent to 0
    t_in_loop = (rem((t - t_rampup_y), t_loop_y));
    for i = 1:loop_periods_y
        if t_in_loop <= sum(loop_durations_y(1:i))
            local_time = t_in_loop;
            if i > 1
                % On the first period, time starts at zero anyway, and
                % there is a one to one correlation between local time and
                % t in the first period of rampup. 
                local_time = local_time - sum(loop_durations_y(1:(i-1)));
            end
            y = polyval(pp_data.get_loop_dpoly(1, 'y', i), local_time);
            vy = polyval(pp_data.get_loop_vpoly(1, 'y', i), local_time);
            ay = polyval(pp_data.get_loop_apoly(1, 'y', i), local_time);
            jy = polyval(pp_data.get_loop_jpoly(1, 'y', i), local_time);
            break
        end
    end
end
end
