%% Sample code to run experiment from script,
% specifying a fixed input signal and recording the measured
% outputs to a variable in the Workspace for further processing.

close;clear;clc;

% Sample rate in sec.
h = 0.02;

% Experiment duration in sec. 
% (don't forget to change this in your diagram, see video)
Tsim = 10;

% Time vector (don't forget to transpose with ')
t = [0:h:Tsim]';

% Input vector
amplitude = 0;%0.02;
omega = 1;
u = amplitude * sin(omega * t);

% Variable that goes to Simulink
% (First column: time, Second column: input values)
simin = [t, u];

%% Start experiment
sim qubetemplate_altered

%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)

% If output type 'Timeseries'
%y = simout.Data;

% If output type 'Array' 
% y = simout;

%% Plot data
plot(t, alpha(:,2), t, theta(:,2))
legend('alpha', 'theta')