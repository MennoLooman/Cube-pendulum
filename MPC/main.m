%% Sample code to run experiment from script,
% specifying a fixed input signal and recording the measured
% outputs to a variable in the Workspace for further processing.

clear;clc;
addpath('..');
hwinit;
simulate_LQR = 0; %[0 run / 1 simulate]
stable_equi = 0; %[0 unstable / 1 stable]
MPC_script;

% Sample rate in sec.
h = 0.005;

% Experiment duration in sec. 
% (don't forget to change this in your diagram, see video)
Tsim = 20;

% Time vector (don't forget to transpose with ')
t = [0:h:Tsim]';
N = Tsim/h; %N+1 samples

%% Start experiment
% load reference
reference = zeros(N+1,1);
reference_signal = timeseries(reference,t);

% load reference for reference tracking
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(reference_signal,'reference');

% sim qubetemplate_with_kalman_blocks_2020b
disp("Running Simulation Now");
sim qubetemplate_kalman_and_LQR_2020b


%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)

y = y_out.data;
u = u_out.data;
x_hat = x_hat_out.data;