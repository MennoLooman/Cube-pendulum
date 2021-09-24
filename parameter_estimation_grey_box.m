%% Load data
load('data/run1.mat');
t = u(:,1);
h = t(2)-t(1);
u = u(:,2);
y = y;
data = iddata(y,u,h); % data file used for grey_est and pem

%% Get linearized system
%linearization;
clearvars -except data u y t h Ac Bc
%guess physical parameters (stable equi)
g1_init = -1;%-r_m/r_p
g2_init = 1;%-damp_motor/I_m
g3_init = -1;%-(g*(m_p*r_m^2 + I_m))/(I_m*r_p)
g4_init = 1;%(g*m_p*r_m)/I_m
g5_init = 1;%motor_constant/I_m 

parameters = {'g1',g1_init;'g2',g2_init;'g3',g3_init;'g4',g4_init;'g5',g5_init};
init_sys = idgrey(@sys_matrices,parameters,'c');%,{},0);

% g6_init = ;%(m_p*r_m^2+I_m)/(m_p*r_p^2)
% g7_init = ;%-damp_pendulum/I_m
% parameters = {'g1',g1_init;'g2',g2_init;'g3',g3_init;'g4',g4_init;'g5',g5_init;'g6',g6_init;'g7',g7_init};
% init_sys = idgrey(@sys_matrices2,parameters,'c');%,{},0);

%% Identification
%interesting options: 'EnforceStability' 'MaxIterations' 
opt_id = greyestOptions('InitialState','zero','Display','on');
%Grey box estimation
[sys_ge, x0_ge] = greyest(data,init_sys,opt_id);
%PEM to refine model
sys_pem = pem(data,sys_ge,opt_id); %use same opt, data as greyest. 

%% Simulate identified model
opt_sim = simOptions('AddNoise',true);
y = sim(sys,udata,opt_sim);

%% Linearized system
%original linearization
function [A,B,C,D] = sys_matrices(g1,g2,g3,g4,g5,Ts) %Ts is needed for greyest
    A = [0 g1*g2 g3 0; 
         0 g2    g4 0;
         eye(2)  zeros(2)];
    B = [g1*g5; 
         g5; 
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
end

%added pendulum resistance
function [A,B,C,D] = sys_matrices2(g1,g2,g3,g4,g5,g6,g7,Ts) %Ts is needed for greyest
    A = [g6*g7 g1*g2 g3 0; 
         g1*g7 g2    g4 0;
         eye(2)      zeros(2)];
    B = [g1*g5; 
         g5;
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
end