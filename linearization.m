clear; clc;

%% load 
%x = [th1d al1d th1 al1]' 
EulerLagrange_script;
clearvars -except MNQF_para

%linearize around: 
th1d_eq = 0;
al1d_eq = 0;
th1_eq = 0;
al1_eq = 0;

x = [th1d al1d th1 al1];
x_d = [MNQF_para',th1d,al1d]; %dynamics

%% Linearize
A_sym = jacobian(x_d , x);
B_sym = jacobian(x_d, voltage);

Ac = subs(A_sym,{th1d, al1d, th1, al1},{th1d_eq,al1d_eq,th1_eq,al1_eq});
Bc = subs(B_sym,{th1d, al1d, th1, al1},{th1d_eq,al1d_eq,th1_eq,al1_eq});