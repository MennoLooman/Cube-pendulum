%% System description
%States and Parameters:
syms th1 th1d th1dd al1 al1d al1dd real1 %states
syms m_p r_p r_m I_p I_m s_m g real1 motor_constant voltage damp_motor damp_pendulum %parameters
q   = [th1       al1   ]';
qd  = [th1d      al1d  ]';
qdd = [th1dd     al1dd ]';

%% Declear real variables
assume(m_p, 'real');
assume(r_p, 'real');
assume(r_m, 'real');
assume(s_m, 'real');
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
P = r_p*m_p*g*(1-cos(th1)) + ...            %pendulum potential
    0.5*al1*s_m*al1;                        %spring wire motor

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
MNQF_para = M^-1*NQF;

% %function for evaluating:
substitute_this =   {I_m,   damp_motor, damp_pendulum,  g,      m_p,    motor_constant, r_m,    r_p,    s_m};
for_this =          {I_m_v, d_m_v,      d_p_v,          g_v,    m_p_v,  m_c_v,          r_m_v,  r_p_v,  s_m_v};
MNQF = matlabFunction(subs(MNQF_para,substitute_this,for_this));

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











