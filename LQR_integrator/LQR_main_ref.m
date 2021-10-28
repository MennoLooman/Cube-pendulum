%% Sample code to run experiment from script,
% specifying a fixed input signal and recording the measured
% outputs to a variable in the Workspace for further processing.

clear;clc;

hwinit;
simulate_LQR = 0; %[0 run / 1 simulate]
stable_equi = 0; %[0 unstable / 1 stable]
LQR_script;

% Sample rate in sec.
h = 0.005; %200Hz, works smoother than 100 Hz

% Experiment duration in sec. 
% (don't forget to change this in your diagram, see video)
Tsim = 15;

% Time vector (don't forget to transpose with ')
t = [0:h:Tsim]';
N = Tsim/h; %N+1 samples

% Input vector
% amplitude = 0.01;%0.02;
% omega = 5;
% u = sin(t*omega)*amplitude;
% input_V = timeseries(u,t);
% 
% %simulink stuff
% ds = Simulink.SimulationData.Dataset;
% ds = ds.addElement(input_V,'in1_signal');

% Variable that goes to Simulink
% (First column: time, Second column: input values)
%simin = [t, u, input_V];

%% Start experiment
% build reference signal
amplitude_ref = 0.8;%0.3;
omega_ref = 0.5;%0.5;
reference = zeros(N+1,1);
reference1 = [zeros(5/h+1,1);zeros(5/h,1);ones(5/h,1);-ones(5/h,1)]*amplitude_ref; %block signal
reference2 = [ zeros(7/h,1) ;sin(omega_ref* t(1:end-7/h))] * amplitude_ref;
reference_signal = timeseries(reference,t);

% load reference for reference tracking
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(reference_signal,'reference');

disp("Running Simulation Now");
sim qubetemplate_kalman_and_LQR_2020b


%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)

% y = [theta(:,2) alpha(:,2)];
y = y_out.data;
u = u_out.data;
x_hat = x_hat_out.data;

%% Plot data
%{
figure(1);
plot(t, x_hat(:,1), t,x_hat(:,2), t,x_hat(:,3) ,t,x_hat(:,4))
legend('theta_d-hat', 'alpha_d-hat', 'theta-hat', 'alpha-hat');
xlim([4.8,Tsim])
%}
%{
figure(2);
clf
hold on
%yyaxis left
ylabel('Measurements [rad]');
plot(t, y(:,1),'LineWidth',2);
plot(t, y(:,2),'LineWidth',2);
plot(t,reference_signal.data,'LineWidth',2);
xlim([6.0,Tsim])

yyaxis right
plot(t,u,'-','LineWidth',1.5)
legend('theta', 'alpha','reference - alpha','u','Location','northeast');%,'southwest');%

xlabel('time [t]');
ylabel('Input voltage u [V]');
title('Disturbance LQR around stable equilibrium');
%}
figure(2);
clf
hold on

title("input and output for stable LQR-I with disturbance")
xlabel("time [s]");

plot(t, y(:,1), t, y(:,2))
ylabel("angle [rad]");

yyaxis right
plot(t,u)
ylabel("input u [-]");

legend('theta', 'alpha','u')
xlim([5.0,10])

ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';
%{
figure(3);
plot(t,y(:,1), t, y(:,2),t,x_hat(:,3) ,t,x_hat(:,4))
legend('theta', 'alpha','theta_h_a_t', 'alpha_h_a_t');
xlim([4.8,Tsim])
%}
%% Save data
% save('data/AB/r5_AB.mat','u','y')