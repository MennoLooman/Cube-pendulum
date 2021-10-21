clear;clc;
%% settings
simulate_LQR = 0; %[0 run / 1 simulate]
stable_equi = 0; %[0 unstable / 1 stable]

t_end = 15;
h = 0.005;
t = 0:h:t_end;
int_gain = 0.35;

dist_moment = 1;
dist_magn = 0.0;

ref_magn = 0.1;
%reference = zeros(2,t_end/h+1);
reference = [ zeros(1,15/h+1); zeros(1,1/h),ref_magn*ones(1,5/h),-ref_magn*ones(1,5/h),zeros(1,4/h+1)];

if ~stable_equi
    offset = [pi*ones(1,t_end/h+1);zeros(1,t_end/h+1)];
else
    offset = zeros(2,t_end/h+1);
end
LQR_script;

% get non-linear equations from euler-lagrange from euler_lagrange
comparison_euler_lagrange;

%% run simulation based on non-linear equations
x0 = [0, 0, pi, 0]; %initial conditions %[theta_d alpha_d theta alpha]

dist_step = dist_moment/h;

%state evolution
x_nonlin = zeros(4,t_end/h+1); %
x_nonlin(:,1) = x0;
%output
y = zeros(2,t_end/h+1);
y_obs = zeros(2,t_end/h+1);
%state estimation
x_hat = zeros(4,t_end/h+1); %
x_hat(:,1) = [0,0,0,0];
%input
u = zeros(1,size(t,2));
%integral action
int_act = zeros(1,size(t,2));

%loop
for i = 1:t_end/h
%input
    u(i) = -K_lqr*x_hat(:,i) + int_act(i)*int_gain;
    if(u(i) > 1)
        u(i) = 1;
    end
    if(u(i) < -1)
        u(i) = -1;
    end
%state evolution
    x_nonlin(1,i+1) = [1 0]*MNQF( x_nonlin(4,i),x_nonlin(2,i),x_nonlin(3,i),x_nonlin(1,i),u(i) )*h + x_nonlin(1,i);   % theta_d
    x_nonlin(2,i+1) = [0 1]*MNQF( x_nonlin(4,i),x_nonlin(2,i),x_nonlin(3,i),x_nonlin(1,i),u(i) )*h + x_nonlin(2,i);   % alpha_d
    x_nonlin(3,i+1) = x_nonlin(1,i)*h + x_nonlin(3,i);                                                                % theta
    x_nonlin(4,i+1) = x_nonlin(2,i)*h + x_nonlin(4,i);                                                                % alpha
    if i == dist_step
        x_nonlin(3,i+1) = x_nonlin(3,i+1) + dist_magn;
    end
%output
    y(:,i) = Cd*x_nonlin(:,i)  - offset(:,i);
    y_obs(:,i) = y(:,i) - reference(:,i);
%state estimation
    x_hat(:,i+1) = (Ad - Kd*Cd)*x_hat(:,i) + Bd*u(i) + Kd*y_obs(:,i);
%integral action
    int_act(i+1) = int_act(i) + (x_hat(4,i) - 0)*h;
end

%% plot figure
figure(11)
clf
subplot(3,1,1);
title("Block reference tracking - simulation of nonlinear equations - unstable");
hold on
plot(t,x_nonlin(1,:),'DisplayName','x-theta_d');
plot(t,x_nonlin(2,:),'DisplayName','x-alpha_d');
legend
xlabel("time [s]");
ylabel("angular velocity [rad/s]");

subplot(3,1,2);
hold on
plot(t,y(1,:),'DisplayName','y-theta');
plot(t,y(2,:),'DisplayName','y-alpha');
%plot(dist_moment+h,y(1,dist_step+1),'ok','DisplayName','set disturbance');
plot(t,reference(2,:),'Color',[0.4940 0.1840 0.5560],'DisplayName','reference signal');
legend
xlabel("time [s]");
ylabel("angle [rad]");

subplot(3,1,3);
hold on
xlabel("time [s]");
ylabel("input u [-]");
plot(t,u,'DisplayName','input u');

yyaxis right
ylabel("alpha error integral [rad s]");
plot(t,int_act,'DisplayName','alplha-error integral');
legend
ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';
%{
figure(13)
plot(t,x_hat);
legend("theta_d","alpha_d","theta","alpha");

figure(14)
plot(t,x_hat(4,:));
%}