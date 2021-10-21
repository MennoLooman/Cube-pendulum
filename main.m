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
amplitude = 0.017;%0.02;
omega = 5;
%u = ([10*h:h/Tsim:1+10*h]'+0.1) .* sin([10*h:h/Tsim*20:20+10*h]' .* t) * amplitude;
%u = ones(size(t))*amplitude;
u = zeros(size(t));
%u = [zeros(1/h,1); sin(20 * (h:h:N/5*h))' ; zeros((Tsim/h - (N/5) - 50)+1,1)] * amplitude;
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
sim qubetemplate


%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)
y = [theta(:,2) alpha(:,2)];
y_cut = y(3.1/h:5/h,:);
t_cut = t(3.1/h:5/h,:);

figure(1)
clf
hold on
plot(t_cut, y_cut(:,1),'-o','DisplayName','theta');
plot(t_cut, y_cut(:,2),'-o','DisplayName','alpha');
xlabel("time [s]");
ylabel("angle [rad]");
title("Pendulum Swinging freely - o for sample");
legend




%% Save data
save('data/AB/r5_AB.mat','u','y')