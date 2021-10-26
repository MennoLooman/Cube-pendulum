close all;clear;clc;

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

disp('The controller can stabilize around the origin or track an reference');
while ~Reference_bool
    Reference_flag  = input('Origin[0] / Reference Step[1] / Reference Sine[2]: ');
    if ~isempty(Reference_flag) && ismember(Reference_flag, [0, 1, 2]); Reference_bool = true; end 
end

%% Summery
disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~');
if MPC_flag; MPC_disp='MPC'; else; MPC_disp='LQR';end
if Integrator_flag; Integrator_disp='I'; else; Integrator_disp='';end
if Unstable_flag; Unstable_disp='unstable'; else; Unstable_disp='stable';end
if Reference_flag; if Reference_flag==2; Reference_disp='while reference tracking an sine wave'; else; Reference_disp='while reference tracking an step function';end; else; Reference_disp='around the origin';end
disp(strcat("Simulating ",MPC_disp,Integrator_disp," controller for the ",Unstable_disp," operation point ",Reference_disp,'.'));

%% Run correct simulation

%% Later
%if Unstable_flag; disp('You will have 5 seconds when simulation starts to put the pendulum in the correct position');end