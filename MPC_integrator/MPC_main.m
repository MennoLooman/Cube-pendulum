%% Sample code to run experiment from script,
% specifying a fixed input signal and recording the measured
% outputs to a variable in the Workspace for further processing.

clear;clc;
hwinit;

% Sample rate in sec.
h = 0.005;

% Experiment duration in sec. 
% (don't forget to change this in your diagram, see video)
Tsim = 20;

% Time vector (don't forget to transpose with ')
t = [0:h:Tsim]';
N = Tsim/h; %N+1 samples

%MPC
stable_equi = 0; %[0 unstable / 1 stable]
%integrator
if stable_equi
    Int_gain = 0.3;
else
    Int_gain = 0.05;%-0.015;
end

MPC_script;

%% Start experiment
% load reference
if stable_equi
    amplitude_ref = 1; % stable
    omega_ref = 2;
else
    amplitude_ref = 0.8;%unstable
    omega_ref = 0.5;
end
reference = zeros(N+1,1);
reference1 = [zeros(5/h+1,1);zeros(5/h,1);ones(5/h,1);-ones(5/h,1)]*amplitude_ref; %block signal
reference2 = [ zeros(7/h,1) ;sin(omega_ref* t(1:end-7/h))] * amplitude_ref;
reference_signal = timeseries(reference2,t);

% load reference for reference tracking
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(reference_signal,'reference');

% sim qubetemplate_with_kalman_blocks_2020b
disp("Running Simulation Now");
sim qubetemplate_kalman_and_MPC_2020b_int

%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)
y = y_out.data;
u = u_out.data;
x_hat = x_hat_out.data;

%% plot
figure(2);
clf
hold on
%yyaxis left
ylabel('Measurements [rad]');
plot(t, y(:,1),'LineWidth',2);
plot(t, y(:,2),'LineWidth',2);
plot(t,reference_signal.data,'LineWidth',2);
xlim([6.0,Tsim])
ylim([-1.1,1.1])
yyaxis right
plot(t,u,'-','LineWidth',2)
legend('theta', 'alpha','reference - alpha','u','Location','southwest');%,'northeast');%
xlim([6.0,Tsim])
ylim([-1.1,1.1])
xlabel('time [s]');
ylabel('Input voltage u [V]');
title('Reference tracking MPCI sine function around unstable equilibrium');
