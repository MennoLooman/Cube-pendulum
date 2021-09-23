%% Sample code to run experiment from script,
% specifying a fixed input signal and recording the measured
% outputs to a variable in the Workspace for further processing.

close;clear;clc;
hwinit;
% Sample rate in sec.
h = 0.02;

% Experiment duration in sec. 
% (don't forget to change this in your diagram, see video)
Tsim = 10;

% Time vector (don't forget to transpose with ')
t = [0:h:Tsim]';
N = Tsim/h; %N+1 samples

% Input vector
amplitude = 0.00;%0.02;
omega = 5;
%u = amplitude * sin(omega * t);
%u = ones(size(t))*amplitude;
u = zeros(size(t));
% u = [zeros(N/10+1,1); 
%     0.5*sin(4 * (h:h:N/5*h))'; 
%     0.6*sin(8 * (h:h:N/5*h))'; 
%     0.8*sin(15 * (h:h:N/5*h))'; 
%     sin(20 * (h:h:N/5*h))'; 
%     -ones(N/50,1); 
%     ones(N/50,1); 
%     -ones(N/50,1); 
%     ones(N/50,1);
%     zeros(N/50,1)]*amplitude;
input_V = timeseries(u,t);

%simulink stuff
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(input_V,'in1_signal');

% Variable that goes to Simulink
% (First column: time, Second column: input values)
simin = [t, u, input_V];

%% Start experiment
sim qubetemplate_altered


%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)

y = [theta(:,2) alpha(:,2)];

%% Plot data
plot(t, y(:,1), t, y(:,2))
legend('theta', 'alpha')

%% Save data
save('data/run6.mat','u','y')