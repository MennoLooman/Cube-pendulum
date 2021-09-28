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
r_m_I = 0.085;
r_p_I = 0.129/2;
d_m_I = 2.059e-3;
d_p_I = 0.0366;
I_m_I = 4.6e-6;
m_p_I = 0.024;
g_I = 9.81;
m_c_I = 0.042/8.4;
%% Get linearized system
%guess physical parameters (stable equi)
% g1_init = -r_m/r_p;
% g2_init = (-d_m/I_m);
% g3_init = (-(g*(m_p*r_m^2 + I_m))/(I_m*r_p));
% g4_init = (g*m_p*r_m)/I_m;
% g5_init = m_c/I_m;
parameters = {'r_m', r_m_I; 'r_p',r_p_I;'d_m',d_m_I;'I_m',I_m_I;'m_p',m_p_I;'g',g_I;'m_c',m_c_I};
%parameters = {'g1',g1_init;'g2',g2_init;'g3',g3_init;'g4',g4_init;'g5',g5_init};
init_sys = idgrey(@sys_matrices,parameters,'c');%,{},0);

% g6_init = (m_p*r_m^2+I_m)/(m_p*r_p^2);
% g7_init = -d_p/I_m;
% parameters = {'g1',g1_init;'g2',g2_init;'g3',g3_init;'g4',g4_init;'g5',g5_init;'g6',g6_init;'g7',g7_init};
% init_sys = idgrey(@sys_matrices2,parameters,'c');%,{},0);

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
function [A,B,C,D] = sys_matrices2(r_m,r_p,d_m,d_p,I_m,m_p,g,m_c,Ts) %Ts is needed for greyest
    A = [(m_p*r_m^2+I_m)/(m_p*r_p^2)*-d_p/I_m -r_m/r_p*(-d_m/I_m) (-(g*(m_p*r_m^2 + I_m))/(I_m*r_p)) 0; 
         -r_m/r_p*-d_p/I_m (-d_m/I_m)    (g*m_p*r_m)/I_m 0;
         eye(2)      zeros(2)];
    B = [-r_m/r_p*m_c/I_m; 
         m_c/I_m;
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
    if Ts>0
        Ac = A;
        A = expm(Ac*Ts);
        B = A\(Ac-eye(4))*B;
    end
end