clear;clc;

%% Genreal settings
h = 0.005;
Tsim = 20; %need: Tsim>wait_time
wait_time = 5;
sec_per_step = 5; % amount of seconds per step (and extra wait)

%% Initial bool
MPC_bool = false;
Unstable_bool = false;
Integrator_bool = false;
Reference_bool = false;

%% Menu options
disp('Qube pendulum control by Chris van der Hoorn and Menno Looman');
disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~');

disp('Run system around the stable or unstable operatrion point');
while ~Unstable_bool
    Unstable_flag   = input('Stable[0] / Unstable[1]: ');
    if ~isempty(Unstable_flag) && ismember(Unstable_flag, [0, 1]); Unstable_bool = true; end
end

disp('Choise between LQR and MPC controller');
while ~MPC_bool
    MPC_flag = input('LQR[0] / MPC[1]: ');
    if ~isempty(MPC_flag) && ismember(MPC_flag, [0, 1]); MPC_bool = true; end 
end

disp('With or without added integrator to reduce steady-state error');
while ~Integrator_bool
    Integrator_flag = input('Original[0] / Integrator[1]: ');
    if ~isempty(Integrator_flag) && ismember(Integrator_flag, [0, 1]); Integrator_bool = true; end 
end

disp('The controller can stabilize around the origin or track a reference');
while ~Reference_bool
    Reference_flag  = input('Origin[0] / Reference Step[1] / Reference Sine[2]: ');
    if ~isempty(Reference_flag) && ismember(Reference_flag, [0, 1, 2]); Reference_bool = true; end 
end

%% Summery
disp('~~~~~~~~~~~~~~~~~~~~~~Summery~~~~~~~~~~~~~~~~~~~~~~~');
if MPC_flag; MPC_disp='MPC'; else; MPC_disp='LQR';end
if Integrator_flag; Integrator_disp='I'; else; Integrator_disp='';end
if Unstable_flag; Unstable_disp='unstable'; else; Unstable_disp='stable';end
if Reference_flag; if Reference_flag==2; Reference_disp='while reference tracking a sine wave'; else; Reference_disp='while reference tracking a step function';end; else; Reference_disp='around the origin';end
disp(strcat("Simulating ",MPC_disp,Integrator_disp," controller for the ",Unstable_disp," operation point ",Reference_disp,'.'));

%% Build reference
if Unstable_flag
    wait = wait_time;
    disp(strcat("You will have ",num2str(wait)," seconds at the start of the simulation to put the pendulum around the unstable operation point"));
    amplitude_ref = 0.8;%unstable
    omega_ref = 1;
else
    wait = 0;
    amplitude_ref = 1; % stable
    omega_ref = 2;
end

t = [0:h:Tsim]';
if Reference_flag
    if Reference_flag==2
        %sine ref
        reference = [ zeros((wait+sec_per_step)/h,1) ;sin(omega_ref* t(1:end-(wait+sec_per_step)/h))] * amplitude_ref;
    else
        %step ref
        dh = sec_per_step/h; %time steps per step
        reference = zeros(Tsim/h+1,1);
        for i = 1:floor((Tsim-wait)/sec_per_step)
            if mod(i,4) < 1
                reference((wait/h+1+dh*(i-1)) : (wait/h+1+dh*i-1)) = -ones(dh,1)*amplitude_ref;
            end
            if mod(i,4) < 3 && mod(i,4)>=2
                reference((wait/h+1+dh*(i-1)) : (wait/h+1+dh*i-1)) = ones(dh,1)*amplitude_ref;
            end
        end
        reference((wait/h+1+dh*floor((Tsim-wait)/sec_per_step)):end) = reference(2*(wait/h+1+dh*floor((Tsim-wait)/sec_per_step))-end-1:(wait/h+dh*floor((Tsim-wait)/sec_per_step)));
    end
else
    %zero ref
    reference = zeros(Tsim/h+1,1);
end
reference_signal = timeseries(reference,t);

%% Run simulink file
hwinit;
if MPC_flag
    MPC_script;
else
    LQR_script;
end

% load reference for reference tracking
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(reference_signal,'reference');

% sim qubetemplate_with_kalman_blocks_2020b
disp("~~~~~~~~~~~~~~~~~Starting Simulink~~~~~~~~~~~~~~~~~~");

if MPC_flag
    sim qubetemplate_kalman_and_MPC_2020b
else
    sim qubetemplate_kalman_and_LQR_2020b
end

%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)
y = y_out.data;
u = u_out.data;
x_hat = x_hat_out.data;

%% Construct title
if Reference_flag
    title_plt1 = "Reference tracking ";
    if Reference_flag==2
        title_plt4 = "sine function ";
    else
        title_plt4 = "step function ";
    end
else
    title_plt1 = "";
    title_plt4 = "origin ";
end
if MPC_flag
    title_plt2 = "MPC";
else
    title_plt2 = "LQR";
end
if Integrator_flag 
    title_plt3 = "I ";
else
    title_plt3 = " ";
end
if Unstable_flag
    title_plt5 = "around unstable equilibrium";
else
    title_plt5 = "around stable equilibrium";
end

%% plot
figure();
clf
hold on
ylabel('Measurements [rad]');
plot(t, y(:,1),'LineWidth',2);
plot(t, y(:,2),'LineWidth',2);
plot(t,reference_signal.data,'LineWidth',2);
xlim([wait,Tsim])
ylim([-1.1,1.1])
yyaxis right
plot(t,u,'-','LineWidth',2)
legend('theta', 'alpha','reference - alpha','u','Location','southwest');%,'northeast');%
xlim([wait,Tsim])
ylim([-1.1,1.1])
xlabel('time [s]');
ylabel('Input voltage u [V]');
title(strcat(title_plt1,title_plt2,title_plt3,title_plt4,title_plt5));

%% TODO's
% TODO Matlab
% - test plot title
% - test integrator
% - check LQR integrator gain

% TODO Simulink:
% - test version integrator

% Things to try:
% - Test Demo file fully
% - increase beta for MPC (instead of integrator)
% - play with place of integrator
% - tune integrator LQR stable & unstable
% - lower frequency h for MPC (and maybe even LQR)