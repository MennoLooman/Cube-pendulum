%clear,clc
%% parameters
gval = 9.81;            % gravity
I_m_val = 1.0*10^-4;    % motor inertia
r_m = 0.12;             % motor to pendulum radius
r_p = 0.06;             % center of gravity of pendulum to axle
m_p = 0.010;            % mass of pendulum
 
damp_motor = 0.01;      % dampening in motor
motor_constant = 0;     % voltage -> torque

%% initial conditions
x0 = [1/4*pi, 1/4*pi, 0, 0]; %[theta alpha theta_d alpha_d]

%% run simscape file
t_stop = 2;
sim physical_dynamics_simulation

theta_t =   ans.theta_sim.time;
theta =     ans.theta_sim.signals.values;
alpha_t =   ans.alpha_sim.time;
alpha =     ans.alpha_sim.signals.values;
theta_d_t = ans.theta_d_sim.time;
theta_d =   ans.theta_d_sim.signals.values;
alpha_d_t = ans.alpha_d_sim.time;
alpha_d =   ans.alpha_d_sim.signals.values;

%% plot output
plot_fig = "on";
if plot_fig == "on"
    figure(11)
    clf
    hold on
    plot(theta_t,theta);
    plot(alpha_t,alpha);
    plot(theta_d_t,theta_d);
    plot(alpha_d_t,alpha_d);
    legend("theta","alpha","theta_d","alpha_d");
    plot(t_v,x(:,1),'m')
    plot(t_v,x(:,2),'b')
    plot(t_v,x(:,3),'r')
    plot(t_v,x(:,4),'g')
end

%% compare to Euler-Lagrange (do not run clear,clc)
rel_diff = zeros(length(theta_t),5); %[t,theta,alpha,theta_d,alpha_d]
rel_diff(:,1) = theta_t;

for i = 1:length(theta_t)
    rel_diff(i,2) = (interp1(t_v,x(:,1),theta_t(i)) - theta(i))/max(theta);
    rel_diff(i,3) = (interp1(t_v,x(:,2),theta_t(i)) - alpha(i))/max(alpha);
    rel_diff(i,4) = (interp1(t_v,x(:,3),theta_t(i)) - theta_d(i))/max(theta_d);
    rel_diff(i,5) = (interp1(t_v,x(:,4),theta_t(i)) - alpha_d(i))/max(alpha_d);
end

plot_fig = "on";
if plot_fig == "on"
    figure(12)
    clf
    plot(rel_diff(:,1),rel_diff(:,2:5));
    legend("theta","alpha","theta_d","alpha_d");
end