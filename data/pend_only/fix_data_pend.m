clear,clc
load('run10.mat');
h = 0.02;
new_t_beg = 1;
new_t_end = 12;

%% get x0;
% x0 = [theta_d, theta]
x0 = [ ( y(round(new_t_beg/h),1)-y(round(new_t_beg/h)-1,1) )/h ,y(round(new_t_beg/h)-1,1)];

%% only take needed part
u_new = u( round(new_t_beg/h):round(new_t_end/h), :);
y_new = y( round(new_t_beg/h):round(new_t_end/h), :);

u = u_new;
y = y_new;

clearvars -except u y x0

save('run10_pend.mat','u','y','x0')