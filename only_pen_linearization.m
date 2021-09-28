%% load 
EulerLagrange_script_only_pendulum;
clearvars -except MNQF_para th1d th1 voltage

x = [th1d th1];
x_d = [MNQF_para',th1d]; %dynamics
stable_equi = 1;  %[0 unstable / 1 stable]

%% linearize around: 
%stable equilibrium:
if stable_equi
    th1d_eq = 0;
    th1_eq = 0;
else 
    th1d_eq = 0;
    th1_eq = pi;
end

%% Linearize
A_sym = jacobian(x_d , x);

Ac = subs(A_sym,{th1d, th1},{th1d_eq,th1_eq});