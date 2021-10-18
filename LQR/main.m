%% Sample code to run experiment from script,
% specifying a fixed input signal and recording the measured
% outputs to a variable in the Workspace for further processing.

close all;clear;clc;
addpath('..');
hwinit;
simulate_LQR = 0; %[0 run / 1 simulate]
stable_equi = 0; %[0 unstable / 1 stable]
LQR_script;

% Sample rate in sec.
h = 0.02;

% Experiment duration in sec. 
% (don't forget to change this in your diagram, see video)
Tsim = 10;

% Time vector (don't forget to transpose with ')
t = [0:h:Tsim]';
N = Tsim/h; %N+1 samples

% Input vector
amplitude = 0.01;%0.02;
omega = 5;
u = sin(t*omega)*amplitude;
input_V = timeseries(u,t);

%simulink stuff
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(input_V,'in1_signal');

% Variable that goes to Simulink
% (First column: time, Second column: input values)
simin = [t, u, input_V];

%% Start experiment
% sim qubetemplate_with_kalman_blocks_2020b
sim qubetemplate_kalman_and_LQR_2020b


%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)

% y = [theta(:,2) alpha(:,2)];
y = y_out.data;
u = u_out.data;
x_hat = x_hat_out.data;

%% Plot data
figure();
plot(t, y(:,1), t, y(:,2))
legend('theta', 'alpha')

figure();
plot(t, x_hat(:,1), t,x_hat(:,2), t,x_hat(:,3) ,t,x_hat(:,4))
legend('theta_d_hat', 'alpha_d_hat', 'theta_hat', 'alpha_hat');

figure();
plot(t,u)
legend('u')
xlim([4.8,6.2])
ylim([-1.5 1.5])

figure();
plot(t,y(:,1), t, y(:,2),t,x_hat(:,3) ,t,x_hat(:,4))
legend('theta', 'alpha','theta_h_a_t', 'alpha_h_a_t');
xlim([4.8,6.2])
%% Save data
% save('data/AB/r5_AB.mat','u','y')