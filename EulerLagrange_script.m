clear,clc
%% System description
%States and Parameters:
syms th1 th1d th1dd al1 al1d al1dd real1 %states
syms m_p r_p r_m I_p I_m g torque real1     %parameters
q   = [th1       al1   ]';
qd  = [th1d      al1d  ]';
qdd = [th1dd     al1dd ]';

%{
%equations
X1 = l*cos(th11);
X2 = l*cos(th11) + l*cos(th11+th12);
Y1 = l*sin(th11);
Y2 = l*sin(th11) + l*sin(th11+th12);

%derivatives pos.
X1d = dt(X1);
Y1d = dt(Y1);
X2d = dt(X2);
Y2d = dt(Y2);
%}
%% Energy bal1ance
%{
%Kinetic energy
K1 = (1/2)*m*(X1d^2+Y1d^2);
K2 = (1/2)*m*(X2d^2+Y2d^2);
K = K1 + K2;

%Potential1 Energy
P1 = m*g*Y1;
P2 = m*g*Y2;
P = P1 + P2;
%}
K = (1/2)*I_p*th1d^2 + (1/2)*m_p*r_m^2*al1d^2 +(1/2)*I_m*al1d^2;
P = r_p*m_p*g*(1-cos(th1));

%% Euler Lagrange equations
%Lagrangian
L = K - P;
L = simplify(L);

%Dampening 
N = [0 0; 0 0];
D = 0;

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
Fal1 = torque;


%% Physical Equations
%Physical1 equations are cal1culated where th11dd and th12dd are expressed as
%function of th1e oth1er states, and parameters.
%th1e result is in th1e form of M*qdd = NQF, or qdd = M^-1*NQF
eqns = [L1th1 - L2th1 == -Dth1 + Fth1
        L1al1 - L2al1 == -Dal1 + Fal1];
vars = [th1dd al1dd];
[M, NQF] = equationsToMatrix(eqns,vars);

return;

%% Simulation
%th1e system of Physical1 equations above will be solved for t, to use th1e
%MATLAB solver, th1e equations must be al1tered to be in th1e proper form.
MNQF = M^-1*NQF;

% First we must set th1e parameters
gval1 = 9.81;   %m/s^2
lval1 = 0.2;    %m
mval1 = 0.050;  %kg
tau1val1 = 0;   %Nm
tau2val1 = 0;   %Nm

%function for eval1uating:
MNQF = matlabFunction(subs(MNQF,[g l m tau1 tau2],[gval1 lval1 mval1 tau1val1 tau2val1])); 
x0 = [-(1/2)*pi, 1/2*pi, 0, 0]; %initial1 conditions
x0sim = [x0(1)+3*pi/2,x0(2:3)];
t_s = 0.001;  %time step
t_end = 3;  %time end
t_v = 0:t_s:t_end;
th1eta = zeros(t_end/t_s+1,4); %[th11(t) th12(t) th11d(t) th12d(t)]
th1eta(1,:) = x0;

%loop
for i = 1:t_end/t_s 
    th1eta(i+1,1) = th1eta(i,3)*t_s+th1eta(i,1);
    th1eta(i+1,2) = th1eta(i,4)*t_s+th1eta(i,2);
    th1eta(i+1,3) = [1 0]*MNQF(th1eta(i,1),th1eta(i,2),th1eta(i,3),th1eta(i,4))*t_s+th1eta(i,3);
    th1eta(i+1,4) = [0 1]*MNQF(th1eta(i,1),th1eta(i,2),th1eta(i,3),th1eta(i,4))*t_s+th1eta(i,4);
end

%plot
%{
figure(1)
plot(t_v,th1eta')
ylim([-50 40])
xlim([0 t_end])
legend("th11","th12","th11d","th12d")
%}

%change theta1 and theta2 to the same frame as th1e simscape:
th1eta_f(:,1) = th1eta(:,1)+3*pi/2;
th1eta_f(:,2) = th1eta(:,2);
th1eta_f(:,3) = th1eta(:,3);
th1eta_f(:,4) = th1eta(:,4);
figure(2)
clf
hold on
plot(t_v,th1eta_f(:,1),'y')
plot(t_v,th1eta_f(:,2),'b')
plot(t_v,th1eta_f(:,3),'r')
plot(t_v,th1eta_f(:,4),'g')
set(gca,'Color','k')

ylim([-50 40])
xlim([0 t_end])
legend("th11","th12","th11d","th12d")

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


