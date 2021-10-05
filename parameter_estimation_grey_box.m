%% Load data
load('data/run6.mat');
t = u(:,1);
h = t(2)-t(1);
u = u(:,2);
y = y;
data = iddata(y,u,h); % data file used for grey_est and pem
%linearization;
clearvars -except data u y t h Ac Bc

%% parameter estimate
r_m_I = 0.085;
%r_p_I = 0.08101;   %0.129/2;
d_m_I = 8e-03;      %2.059e-3;
%d_p_I = 1.731e-05; %3.66e-2
I_m_I = 4.6e-6;
%m_p_I = 0.024;
%g_I = 9.812;
m_c_I = 5e-3;       %0.042/8.4 = 5e-3

%    96,1%
r_p = 0.08101;
d_p = 1.731e-05;
m_p = 0.024;d
g = 9.812;

%% Get linearized system
%guess physical parameters (stable equi)
%parameters = {'r_m', r_m_I; 'r_p',r_p_I;'d_m',d_m_I;'I_m',I_m_I;'m_p',m_p_I;'g',g_I;'m_c',m_c_I};
parameters2 = {'r_m',r_m_I;'d_m',d_m_I;'I_m',I_m_I;'m_c',m_c_I};
%init_sys = idgrey(@sys_matrices,parameters,'c');
init_sys = idgrey(@sys_matrices2,parameters2,'c');

%% Identification
%interesting options: 'EnforceStability' 'MaxIterations' 
opt_id = greyestOptions('InitialState','zero','Display','on','EnforceStability',true);
%opt_id.Regularization.Lambda = 1;
opt_id.Regularization.R = [1e5, 1e5,1e-3,1,1e-3,1e3,1e5,1e-3];
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
function [A,B,C,D] = sys_matrices(r_m,r_p,d_m,I_m,m_p,g,m_c,Ts) %Ts is needed for greyest
    A = [0 -r_m/r_p*(-d_m/I_m) (-(g*(m_p*r_m^2 + I_m))/(I_m*r_p)) 0; 
         0 (-d_m/I_m)    (g*m_p*r_m)/I_m 0;
         eye(2)  zeros(2)];
    B = [-r_m/r_p*m_c/I_m; 
         m_c/I_m; 
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
%     if Ts>0
%         Ac = A;
%         A = expm(Ac*Ts);
%         B = A\(Ac-eye(4))*B;
%     end
end

%added pendulum resistance
function [A,B,C,D] = sys_matrices2(r_m,d_m,I_m,m_c,Ts) %Ts is needed for greyest
    A = [(0.024*r_m^2+I_m)/(0.024*0.08101^2)*-1.731e-05/I_m, -r_m/0.08101*(-d_m/I_m), (-(9.812*(0.024*r_m^2 + I_m))/(I_m*0.08101)), 0; 
         -r_m/0.08101*-1.731e-05/I_m, (-d_m/I_m),    (9.812*0.024*r_m)/I_m, 0;
         eye(2)      zeros(2)];
    B = [-r_m/0.08101*m_c/I_m; 
         m_c/I_m;
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
end