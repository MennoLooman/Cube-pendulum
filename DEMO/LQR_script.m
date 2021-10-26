%% COST TERMS
%Tuneing
if(~Unstable_flag)
    %states:
        Q1=1e5;     %theta_d
        Q2=1e-3;    %alpha_d
        Q3=1e6;     %theta
        Q4=1e6;     %alpha
        Q_lqr = diag([Q1,Q2,Q3,Q4]);
    %input
        R_lqr = 1e-10;
    %cross-terms
        N_lqr = zeros(size(Q_lqr,1),size(R_lqr,2));
else %unstable
    %states:
        Q1=1e6; %theta_d
        Q2=5e4; %alpha_d
        Q3=1e8; %theta
        Q4=5e6; %alpha
        Q_lqr = diag([Q1,Q2,Q3,Q4]);
    %input
        R_lqr = 1e-10;
    %cross-terms
        N_lqr = zeros(size(Q_lqr,1),size(R_lqr,2));
end

%% get system matrices and kalm-filter observer
observer_script;

%% make state-feedback LQR controller
Co = ctrb(Ac,Bc);
if( length(Ac)-rank(Co) ~= 0)
    disp("system is not controllable.");
    return
else
    disp("system is controllable.");
end
[K_lqr,~,~] = dlqr(Ad,Bd,Q_lqr,R_lqr,N_lqr);