%% Load data
load('data/run4.mat');
t = u(:,1);
h = t(2)-t(1);
u = u(:,2);
y = y;
data = iddata(y,u,h); % data file used for grey_est and pem
%linearization;
clearvars -except data u y t h Ac Bc

%% parameter estimate
r_p_I = 0.129/2;
d_p_I = 0.0366;
m_p_I = 0.024;
g_I = 9.81;
%% Get linearized system
%guess physical parameters (stable equi)
parameters = {'r_p',r_p_I;'d_p',d_p_I;'m_p',m_p_I;'g',g_I};
init_sys = idgrey(@sys_matrices,parameters,'c');%,{},0);

%% Identification
%interesting options: 'EnforceStability' 'MaxIterations' 
opt_id = greyestOptions('InitialState','zero','Display','on');
%opt_id.Regularization.Lambda = 1;
opt_id.Regularization.R = [1e5, 1e5,1e-3,1,1,1e5,1];
%opt_id.Regularization.Nominal = 'model';
%opt_id.OutputWeight = [1e-5 0; 0 1e5];

%Grey box estimation
[sys_ge, x0_ge] = greyest(data,init_sys,opt_id);
%PEM to refine model
sys_pem = pem(data,sys_ge,opt_id); %use same opt, data as greyest. 
%TODO: check is greyest already does PEM
%% Simulate identified model
opt_sim = simOptions('AddNoise',false);
y_ge = sim(sys_ge,data,opt_sim);
y_pem = sim(sys_pem,data,opt_sim);

%% Linearized system
%original linearization
function [A,B,C,D] = sys_matrices(r_p,d_p,m_p,g,Ts) %Ts is needed for greyest
    A = [-d_p/(m_p*r_p^2) -g/r_p;
         1                0    ];
    B = [0;0];
    C = [0 1];
    D = [0;0];
end