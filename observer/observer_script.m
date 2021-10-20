%% Observer Parameters
%needs a value, but if ran from another script, don't overwrite.
if ~exist('h','var')
    h = 0.02; 
end
%check if stability was already defined, otherwise:
if ~exist('stable_equi','var')
    stable_equi = 1; %[0 unstable / 1 stable]
end

%offset so that pendulum can be initialized downward.
if stable_equi == 1
    theta_offset = 0;
else
    theta_offset = pi;
end
%NOISE TERMS
%state:
    Q1 = 1;    %theta_d
    Q2 = 1;    %alpha_d
    Q3 = 0.001;    %theta
    Q4 = 0.01;    %alpha
    Q_kal = diag([Q1,Q2,Q3,Q4]);
%output:
    R1 = 0.00005;     %theta
    R2 = 0.00005;     %theta
    R_kal = diag([R1,R2]);
%cross-terms:
    S = zeros(size(Q_kal,1),size(R_kal,2));

%% parameters obtained by estimation
% run: /data/AB_new/AB_new5_short.mat
% highest validatin score from pendulum -> whole-system method, also 'reasonable' physical values.
r_p_v = 0.0806; 
d_p_v = 2.9e-05; 
m_p_v = 0.024;
g_v   = 9.812;
r_m_v = 0.082504; 
d_m_v = 3.317e-13;
I_m_v = 0.0002036;
m_c_v = 0.21872; 
s_m_v = 0.0025753;

x0 = [0;0;0;0];

%% getting state matrices
%system matrices depend on what operating point we linearize about.
if stable_equi == 1
    [Ac,Bc,Cc,Dc] = sys_matrices_stable(r_p_v,d_p_v,m_p_v,g_v,r_m_v,d_m_v,I_m_v,m_c_v,s_m_v); %system matrices
else
    [Ac,Bc,Cc,Dc] = sys_matrices_unstable(r_p_v,d_p_v,m_p_v,g_v,r_m_v,d_m_v,I_m_v,m_c_v,s_m_v); %system matrices
end
sys_c = ss(Ac,Bc,Cc,Dc); %continuous time system

%check observability
Ob = obsv(Ac,Cc);
if( length(Ac)-rank(Ob) ~= 0)
    disp("system is not observable.");
    return
else
    disp("system is observable.");
end

%% making observer
sys_d = c2d(sys_c,h);
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

%[X,K,L] = idare(A,B,Q,R,S,E)
[~,Kd,~] = idare(Ad',Cd',Q_kal,R_kal,S,eye(size(Ad,1)));
Kd = Kd';

%% using observer to estimate states (not live in simulink)
%% plot estimates
if ~exist('plot_figure','var')
    plot_figure = 1;
end
if plot_figure
    
    load('../data/AB_new/AB_new6_long.mat');
    t = u(:,1);
    u = u(:,2);

    x_hat = zeros(size(Ad,1),size(y,1));
    y_hat = zeros(size(Cd,1),size(y,1));
    x_hat(:,1) = x0';

    for k=1:size(y,1)-1
        x_hat(:,k+1) = (Ad - Kd*Cd)*x_hat(:,k) + Bd*u(k) + Kd*y(k,:)';
        y_hat(:,k) = Cd*x_hat(:,k);
    end


    figure(11);
    clf
    title("Kalman filter on test-run outputs");
    hold on
    plot(t,x_hat(1,:),'DisplayName','x-theta_d')
    plot(t,x_hat(2,:),'DisplayName','x-alpha_d')
    plot(t,x_hat(3,:),'DisplayName','x-theta')
    plot(t,x_hat(4,:),'DisplayName','x-alpha')
    % plot outputs
    plot(t,y(:,1),'DisplayName','y-theta')
    plot(t,y(:,2),'DisplayName','y-alpha')
    legend
    ylim([-0.3,0.5])
end
        
%% functions
function [A,B,C,D] = sys_matrices_stable(r_p,d_p,m_p,g,r_m,d_m,I_m,m_c,s_m)
    A = [-d_p*(m_p*r_m^2+I_m)/(I_m*m_p*r_p^2) d_m*r_m/(I_m*r_p) -g*(m_p*r_m^2 + I_m)/(I_m*r_p) (r_m*s_m)/(I_m*r_p); 
         r_m*d_p/(I_m*r_p)                    -d_m/I_m          (g*m_p*r_m)/I_m                 -s_m/I_m;
         eye(2)      zeros(2)];
    B = [-m_c*r_m/(I_m*r_p); 
         m_c/I_m;
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
end

function [A,B,C,D] = sys_matrices_unstable(r_p,d_p,m_p,g,r_m,d_m,I_m,m_c,s_m)
    A = [-d_p*(m_p*r_m^2+I_m)/(I_m*m_p*r_p^2) -d_m*r_m/(I_m*r_p) g*(m_p*r_m^2 + I_m)/(I_m*r_p) -(r_m*s_m)/(I_m*r_p); 
         -r_m*d_p/(I_m*r_p)                    -d_m/I_m          (g*m_p*r_m)/I_m                 -s_m/I_m;
         eye(2)      zeros(2)];
    B = [m_c*r_m/(I_m*r_p); 
         m_c/I_m;
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
end