%% Load data
load('data/run8.mat');
t = u(:,1);
h = t(2)-t(1);
u = u(:,2);
y = y(:,1);
data = iddata(y,[],h); % data file used for grey_est and pem

%% Initial guess
r_p = 0.129/2;%0.08101;
d_p = 1.731e-4;%1.731e-5;
m_p = 0.024;
g = 9.8;%9.812;

%% System
order = [1 0 2]; %[Ny Nu Nx]
parameters = {r_p,d_p,m_p,g};
%initial_states = 
%Ts = 0;
init_sys = idnlgrey('sys_nonlin_pen',order,parameters);

setpar(init_sys,'Fixed',{false false true true});

%% Identification
sys_nl_ge = nlgreyest(data,init_sys,'Display','Full');
