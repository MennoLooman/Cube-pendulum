clear,clc
load('AB_new1.mat');
h = 0.02;
new_t_beg = 0.9;
new_t_end = 4.9;

t1 = u(:,1);
u1 = u(:,2);
y1 = y;

figure(1);
clf;
hold on;
plot(t1,u1, 'DisplayName', 'input u');
plot(t1,y1(:,1), 'DisplayName', 'theta y');
plot(t1,y1(:,2), 'DisplayName', 'alpha y');
hold off;
legend;

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

save('AB_new1_long.mat','u','y','x0')