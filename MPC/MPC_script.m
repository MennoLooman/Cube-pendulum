%% settings
if ~exist('h','var')
    h = 0.005; %based on rise time of pendulum swing (also used during estimation, but could now independently altered here.
end

if ~exist('stable_equi','var')
    stable_equi = 1;
end

%tuning
if(stable_equi)
    %settings stable equi
    Horizon_P = 2; %prediction horizon
    Horizon_C = 1; %controller horizon
    Q = diag([1e6 5e4 1e8 5e6]); %Q tuning states 4x4
    R = 1e-10; %R tuning input 1x1
    beta = 1; %weight on terminal cost
    alpha_bound = 1.4; %upper and lower bound for alpha
else
    %settings unstable equi
    Horizon_P = 3; %prediction horizon
    Horizon_C = 1; %controller horizon
    Q = diag([1e6 5e4 1e8 5e6]); %Q tuning states 4x4
    R = 1e-10; %R tuning input 1x1
    beta = 1; %weight on terminal cost
    alpha_bound = 1.4; %upper and lower bound for alpha
end

%% get system matrices and kalm-filter observer
addpath '..\observer'
plot_figure = 0;
observer_script;

%% make MPC controller
Co = ctrb(Ac,Bc);
if( length(Ac)-rank(Co) ~= 0)
    disp("system is not controllable.");
    return
else
    disp("system is controllable.");
end

% get Penalty matrix P for final state
[P_are,~,~,P_info] = idare(Ad,Bd,Q,R,[],[]);
P = beta * P_are; % used for: 1/2 x(N)' P x(N)

if P_info.Report > 0
   error("solve_DARE: DARE not solved properly"); 
end

%sys behaviour and initial condition: Aeq xu = beq
Aeq = [[eye(4*(Horizon_P+1)) + kron(diag(ones(1,Horizon_P),-1),-Ad)] [zeros(4,Horizon_P); kron(eye(Horizon_P),-Bd)]];
%note beq comes from MPC_controller.m

%constraints: Aleq xu <= beq
Aleq = [[zeros(2*Horizon_P,4*(Horizon_P+1)) kron(eye(Horizon_P),[1;-1])];
        [zeros(2*Horizon_P,4) kron(eye(Horizon_P),[0 0 0 1; 0 0 0 -1]) zeros(2*Horizon_P,Horizon_P)]];
bleq = [ones(2*Horizon_P,1); alpha_bound*ones(2*Horizon_P,1)];

%Objective function 
Objective_H = blkdiag(kron(eye(Horizon_P),Q) ,P, kron(eye(Horizon_P),R));