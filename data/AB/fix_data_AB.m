clear,clc
load('r1_AB.mat');
h = 0.02;
new_t_beg = 0.9;
new_t_end = 1.6;

%% get x0;
% x0 = [theta_d, alpha_d, theta, alpha]
x0 = [ ( y(round(new_t_beg/h),1)-y(round(new_t_beg/h)-1,1) )/h , ( y(round(new_t_beg/h),2)-y(round(new_t_beg/h)-1,2) )/h ,...
       y(round(new_t_beg/h),1) , y(round(new_t_beg/h),2)];

%% only take needed part
u_new = u( round(new_t_beg/h):round(new_t_end/h), :);
y_new = y( round(new_t_beg/h):round(new_t_end/h), :);

u = u_new;
y = y_new;

clearvars -except u y x0

save('data/AB/r1_AB_short2.mat','u','y','x0')