%% Set Functions and Time Range
clear
t_initial = 0;
t_final = 2;
resolution = 0.001;
path_planner = 'path_planning_level1_v10'; %LEVEL 1
%path_planner = 'path_planning_level2_v02'; %LEVEL 2
path_gen_hand1 = 'arm1path_level1_v10';    %LEVEL 1
%path_gen_hand1 = 'arm1path_level2_v01';   %LEVEL 2
%path_gen_hand2 = 'arm2path_level2_v01';   %LEVEL 2

%% Create Function Handles
path_plan_fun = str2func(path_planner);
path_gen_fun_hand1 = str2func(path_gen_hand1);  % Both Levels
%path_gen_fun_hand2 = str2func(path_gen_hand2); %LEVEL 2
pp_data = path_plan_fun(t_final);

%% Loop for Hand1
t = t_initial:resolution:(t_final);
for i = 1:length(t)
    [x, y, v_x, v_y, a_x, a_y, j_x, j_y] = path_gen_fun_hand1(pp_data, t(i));
    y_(i) = y;
    vy(i) = v_y;
    ay(i) = a_y;
    jy(i) = j_y;
    x_(i) = x;
    vx(i) = v_x;
    ax(i) = a_x;
    jx(i) = j_x;
end


figure(2)
subplot(4,1,1)       % add first plot in 2 x 1 grid
plot(t,y_,'-k')
title('Position Y - Hand1')

subplot(4,1,2)       % add first plot in 2 x 1 grid
plot(t,vy)
title('Velocity Y - Hand1')

subplot(4,1,3)       % add second plot in 2 x 1 grid
plot(t,ay,'-r')       % plot using + markers
title('Acceleration Y - Hand1')

subplot(4,1,4)       % add second plot in 2 x 1 grid
plot(t,jy,'-o')       % plot using + markers
title('Jerk Y - Hand1')
xlabel('Time (s)')

figure(3)
subplot(4,1,1)       % add first plot in 2 x 1 grid
plot(t,x_,'-k')
title('Position X - Hand1')

subplot(4,1,2)       % add first plot in 2 x 1 grid
plot(t,vx)
title('Velocity X - Hand1')

subplot(4,1,3)       % add second plot in 2 x 1 grid
plot(t,ax,'-r')       % plot using + markers
title('Acceleration X - Hand1')

subplot(4,1,4)       % add second plot in 2 x 1 grid
plot(t,jx,'-o')       % plot using + markers
title('Jerk X - Hand1')
xlabel('Time (s)')



%% Loop for Hand2
t = t_initial:resolution:(t_final);
for i = 1:length(t)
    [x, y, v_x, v_y, a_x, a_y, j_x, j_y] = path_gen_fun_hand2(pp_data, t(i));
    y_(i) = y;
    vy(i) = v_y;
    ay(i) = a_y;
    jy(i) = j_y;
    x_(i) = x;
    vx(i) = v_x;
    ax(i) = a_x;
    jx(i) = j_x;
end


figure(4)
subplot(4,1,1)       % add first plot in 2 x 1 grid
plot(t,y_,'-k')
title('Position Y - Hand2')

subplot(4,1,2)       % add first plot in 2 x 1 grid
plot(t,vy)
title('Velocity Y - Hand2')

subplot(4,1,3)       % add second plot in 2 x 1 grid
plot(t,ay,'-r')       % plot using + markers
title('Acceleration Y - Hand2')

subplot(4,1,4)       % add second plot in 2 x 1 grid
plot(t,jy,'-o')       % plot using + markers
title('Jerk Y - Hand2')
xlabel('Time (s)')

figure(5)
subplot(4,1,1)       % add first plot in 2 x 1 grid
plot(t,x_,'-k')
title('Position X - Hand2')

subplot(4,1,2)       % add first plot in 2 x 1 grid
plot(t,vx)
title('Velocity X - Hand2')

subplot(4,1,3)       % add second plot in 2 x 1 grid
plot(t,ax,'-r')       % plot using + markers
title('Acceleration X - Hand2')

subplot(4,1,4)       % add second plot in 2 x 1 grid
plot(t,jx,'-o')       % plot using + markers
title('Jerk X - Hand2')
xlabel('Time (s)')
