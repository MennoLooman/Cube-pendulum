%% System description
%States and Parameters:
syms th1 th1d th1dd al1 al1d al1dd real1 %states
syms m_p r_p r_m I_p I_m g real1 motor_constant voltage damp_motor damp_pendulum %parameters
q   = [th1       al1   ]';
qd  = [th1d      al1d  ]';
qdd = [th1dd     al1dd ]';

%% Declear real variables
assume(m_p, 'real');
assume(r_p, 'real');
assume(r_m, 'real');
assume(I_p, 'real');
assume(I_m, 'real');
assume(g, 'real');
assume(motor_constant, 'real');
assume(voltage, 'real');
assume(damp_motor, 'real');
assume(damp_pendulum, 'real');

%% Energy bal1ance
%{
Old (polar)
K = (1/2)*I_p*th1d^2 + ...                              %pendulum theta
    (1/2)*m_p*(r_m^2 + (sin(th1)*r_p)^2)*al1d^2 + ...   %pendulum alpha
    (1/2)*I_m*al1d^2;                                   %motor alpha
P = r_p*m_p*g*(1-cos(th1));
%}
%{
conference paper (cartesian)
K = (1/2)*I_m*th1d^2 + ...                              %pendulum theta
    (1/2)*m_p*((r_p*th1d-r_m*cos(al1)*al1d)^2 + (-r_m*sin(al1)*al1d)^2  + (r_m*cos(al1)^2*al1d^2)^2  ) + ...   %pendulum alpha
    (1/2)*I_p*al1d^2;                                   %motor alpha
P = r_p*m_p*g*(cos(al1));
%}
%new cartesian
Vx = (-cos(al1)*r_m + sin(al1)*r_p*sin(th1))*al1d -cos(al1)*r_p*cos(th1)*th1d;
Vy = ( sin(al1)*r_m + cos(al1)*r_p*sin(th1))*al1d +sin(al1)*r_p*cos(th1)*th1d;
Vz = sin(th1)*r_p*th1d;

K = (1/2)*m_p*(Vx^2 + Vy^2 + Vz^2) + ...    %pendulum cartesian
    (1/2)*I_m*al1d^2;                       %motor alpha
P = r_p*m_p*g*(1-cos(th1));

%% Euler Lagrange equations
%Lagrangian
L = K - P;
L = simplify(L);

%Dampening 
%N = [0 0; 0 damp_motor];
N = [damp_pendulum 0; 0 damp_motor];
D = 1/2 * qd'*N*qd;
D = simplify(D);

%(d/dt)(dL/dqd)
L1th1 = simplify(dt(diff(L,th1d)));
L1al1 = simplify(dt(diff(L,al1d)));

%(dL/dq)
L2th1 = simplify(diff(L,th1));
L2al1 = simplify(diff(L,al1));

%dD/dqd
Dth1 = simplify(diff(D,th1d));
Dal1 = simplify(diff(D,al1d));

%F
Fth1 = 0;
Fal1 = motor_constant * voltage; %motor_constant = Kr/R

%% Physical Equations
%Physical1 equations are cal1culated where th11dd and th12dd are expressed as
%function of th1e oth1er states, and parameters.
%th1e result is in th1e form of M*qdd = NQF, or qdd = M^-1*NQF
eqns = [L1th1 - L2th1 == -Dth1 + Fth1
        L1al1 - L2al1 == -Dal1 + Fal1];
vars = [th1dd al1dd];
[M, NQF] = equationsToMatrix(eqns,vars);

%% Simulation
%th1e system of Physical equations above will be solved for t, to use the
%MATLAB solver, the equations must be altered to be in the proper form.
MNQF_para = M^-1*NQF;

% % First we must set the parameters
% m_p_val = 0.024; %kg
% r_p_val = 0.129/2; %kg
% r_m_val = 0.085; %m
% % I_p_val = (1/3)*m_p_val*(r_p_val*2)^2; %kg*m^2 %Wordt niet gebruikt in equations.
% I_m_val = 1.0*10^-3; %kg*m^2 ??????
% g_val = 9.81; %m/s^2
% motor_constant_val = 1; %Kr/R
% damp_motor_val = 0.01; %random guess
% 
% %function for evaluating:
% MNQF = matlabFunction(subs(MNQF_para,{m_p r_p r_m I_m g motor_constant damp_motor voltage}, {m_p_val r_p_val r_m_val I_m_val g_val motor_constant_val damp_motor_val 0} )); 
% 
% x0 = [pi+0.00001, 0, 0, 0]; %initial conditions %[theta alpha theta_d alpha_d]
% t_s = 0.001;  %time step
% t_end = 3;  %time end
% t_v = 0:t_s:t_end;
% x = zeros(t_end/t_s+1,4); %
% x(1,:) = x0;
% 
% %loop
% for i = 1:t_end/t_s 
%     x(i+1,1) = x(i,3)*t_s + x(i,1);
%     x(i+1,2) = x(i,4)*t_s + x(i,2);
%     x(i+1,3) = [1 0]*MNQF(x(i,4),x(i,1),x(i,3))*t_s + x(i,3);
%     x(i+1,4) = [0 1]*MNQF(x(i,4),x(i,1),x(i,3))*t_s + x(i,4);
% end
% 
% figure()
% clf
% hold on
% plot(t_v,x(:,1),'m')
% plot(t_v,x(:,2),'b')
% plot(t_v,x(:,3),'r')
% plot(t_v,x(:,4),'g')
% 
% %ylim([-50 40])
% xlim([0 t_end])
% legend("theta","alpha","theta_d","alpha_d")

%% Functions
function res = dt(input)
    %Function to get time derivative. Accepts only if equation does not
    %contain states higher than double dot (this should never be needed for
    %Euler Lagrange in this context), otherwise returns error.
    syms th1 th1d th1dd al1 al1d al1dd real1
    if(has(input,th1dd) || has(input,th1dd))
        disp('Time derivative error');
        return
    end
    res = diff(input,th1)*th1d + diff(input,al1)*al1d + diff(input,th1d)*th1dd + diff(input,al1d)*al1dd;
end


