load('data/run1.mat');
clc
%% get M and NQF
EulerLagrange_script;
clearvars -except MNQF_para h y u
% use the symbolic vars
syms th1 th1d th1dd al1 al1d al1dd real1 %states
syms g
%syms m_p r_p r_m I_p I_m g real1 motor_constant voltage damp_motor  %parameters

y = y(14:end,:);
u = u(14:end,:);

%% make Function
g_val = 9.81; %known
MNQF = matlabFunction(simplify(subs(MNQF_para,{g}, {g_val} )) ) ;
sdpvar m_p r_p r_m I_m motor_constant damp_motor;
%MNQF inputs: MNQF(I_m,al1d,damp_motor,m_p,motor_constant,r_m,r_p,th1,th1d,voltage)
%voltage -> u

%% Optimization
x_hat = zeros(4,1); % x = [theta_d, alpha_d, theta, alpha] 
objective = 0;
for k = 2:size(y,1)
    x_hat(1:2) = x_hat(1:2) + MNQF( I_m,x_hat(2),damp_motor,m_p,motor_constant,r_m,r_p,x_hat(3),x_hat(1),u(k-1,2) )*h;
    % evaluate MNQF for double dots -> theta_d[k] = theta_d[k-1] + theta_dd[k-1]*h
    x_hat(3:4) = x_hat(3:4) + x_hat(1:2)*h;
    % theta[k] = theta[k-1] + theta_d[k-1]*h
    y_hat = [zeros(2),eye(2)] * x_hat; % y = C*x + 0*u;
    
    objective = objective + norm(y(k,:) - y_hat')^2;
end