%% settings
h = 0.02; %based on rise time of pendulum swing (also used during estimation, but could now independently altered here.
% stable_equi = 1; %[0 unstable / 1 stable]
simulate_LQR = 1; %[0 run / 1 simulate]

%COST TERMS
if(stable_equi)
    %states:
        Q1=1e1; %theta_d
        Q2=1e1; %alpha_d
        Q3=1e0; %theta
        Q4=1e3; %alpha
        Q_lqr = diag([Q1,Q2,Q3,Q4]);
    %input
        R_lqr = 1e2;
    %cross-terms
        N_lqr = zeros(size(Q_lqr,1),size(R_lqr,2));
else %unstable
    %states:
        Q1=1e0; %theta_d
        Q2=1e0; %alpha_d
        Q3=1e0; %theta
        Q4=1e0; %alpha
        Q_lqr = diag([Q1,Q2,Q3,Q4]);
    %input
        R_lqr = 1e0;
    %cross-terms
        N_lqr = zeros(size(Q_lqr,1),size(R_lqr,2));
end

%% get system matrices and kalm-filter observer
addpath '..\observer'
plot_figure = 0;
observer_script;

%% make state-feedback LQR controller
Co = ctrb(Ac,Bc);
if( length(Ac)-rank(Co) ~= 0)
    disp("system is not controllable.");
    return
else
    disp("system is controllable.");
end
%[K,S,e] = dlqr(A,B,Q,R,N)
[K_lqr,~,~] = dlqr(Ad,Bd,Q_lqr,R_lqr,N_lqr);

%% simulate the response of the controlled system:
if(simulate_LQR)
sim_len = 5;            %seconds
sim_x0 = [0;0;0;pi/32]; %initial conditions 
%[theta_d,alpha_d,theta,alpha]
t = 0:h:sim_len-h;

%% Linear state evolution:
x_lin = zeros(4,sim_len/h);
y_lin = zeros(2,sim_len/h);
u_lin = zeros(1,sim_len/h);
x_hat_lin = zeros(4,sim_len/h);

x_lin(:,1) = sim_x0;
%x_hat_lin(:,1) = sim_x0;

for i = 1:sim_len/h-1
%input
    u_lin(i) = -K_lqr*x_hat_lin(:,i);
%system evolution
    x_lin(:,i+1) = Ad*x_lin(:,i) + Bd*u_lin(i);
%output
    y_lin(:,i) = Cd*x_lin(:,i);
%state estimation
    x_hat_lin(:,i+1) = (Ad - Kd*Cd)*x_hat_lin(:,i) + Bd*u_lin(i) + Kd*y_lin(:,i);
end

figure(1)
clf

subplot(2,2,1);
hold on
title("State evolution");
plot(t,x_lin(1,:),'DisplayName','x-theta_d');
plot(t,x_lin(2,:),'DisplayName','x-alpha_d');
plot(t,x_lin(3,:),'DisplayName','x-theta');
plot(t,x_lin(4,:),'DisplayName','x-alpha');
legend
ylabel("angle and velocity (rad and rad/s)")
xlabel("time (s)");

%figure(2)
%clf
subplot(2,2,2); 
title("input evolution");
plot(t,u_lin,'DisplayName','input u');
legend
ylabel("input (-)")
xlabel("time (s)");

%figure(3)
%clf
subplot(2,2,3);
hold on
title("States vs state estimate - positions");
plot(t,x_hat_lin(1,:),'DisplayName','x-hat-theta');
plot(t,x_hat_lin(2,:),'DisplayName','x-hat-alpha');
plot(t,x_lin(1,:),'DisplayName','x-theta');
plot(t,x_lin(2,:),'DisplayName','x-alpha');
legend
ylabel("angle and velocity (rad and rad/s)")
xlabel("time (s)");

%figure(4)
%clf
subplot(2,2,4);
title("States vs state estimate - velocities");
hold on
plot(t,x_hat_lin(3,:),'DisplayName','x-hat-theta_d');
plot(t,x_hat_lin(4,:),'DisplayName','x-hat-alpha_d');
plot(t,x_lin(3,:),'DisplayName','x-theta_d');
plot(t,x_lin(4,:),'DisplayName','x-alpha_d');
legend
ylabel("angle and velocity (rad and rad/s)")
xlabel("time (s)");


%% Non-linear state evolution:
%get non-linear system from 
nonLinEquations;
sim_x0 = [0;0;pi/16;0]; %initial conditions

x_nol = zeros(4,sim_len/h); %[theta_d,alpha_d,theta,alpha]
y_nol = zeros(2,sim_len/h);
u_nol = zeros(1,sim_len/h);
x_hat_nol = zeros(4,sim_len/h);

x_nol(:,1) = sim_x0;

x_d = zeros(4,1);
for i = 1:sim_len/h-1
%input
    u_nol(i) = -K_lqr*(x_nol(:,i)-[0;0;theta_offset;0]);
    u_nol(i) = 0;
    
%system evolution
    x_d(1:2) = MNQF(x_nol(4,i),x_nol(2,i),x_nol(3,i),x_nol(1,i),u_nol(i));
    x_d(3:4) = x_nol(1:2,i);
    x_nol(:,i+1) = x_nol(:,i) + x_d*h;
%output    
    y_nol(:,i) = Cd*(x_nol(:,i));
%state estimation
    x_hat_nol(:,i+1) = (Ad - Kd*Cd)*x_hat_nol(:,i) + Bd*u_nol(i) + Kd*y_nol(:,i);
end

figure(2)
clf

subplot(2,2,1);
hold on
title("State evolution");
plot(t,x_nol(1,:),'DisplayName','x-theta_d');
plot(t,x_nol(2,:),'DisplayName','x-alpha_d');
plot(t,x_nol(3,:),'DisplayName','x-theta');
plot(t,x_nol(4,:),'DisplayName','x-alpha');
legend
ylabel("angle and velocity (rad and rad/s)")
xlabel("time (s)");
%sylim([-3.5,3.5]);

subplot(2,2,2); 
title("input evolution");
plot(t,u_nol,'DisplayName','input u');
legend
ylabel("input (-)")
xlabel("time (s)");

subplot(2,2,3);
hold on
title("States vs state estimate - positions");
plot(t,x_hat_nol(1,:),'DisplayName','x-hat-theta');
plot(t,x_hat_nol(2,:),'DisplayName','x-hat-alpha');
plot(t,x_nol(1,:),'DisplayName','x-theta');
plot(t,x_nol(2,:),'DisplayName','x-alpha');
legend
ylabel("angle and velocity (rad and rad/s)")
xlabel("time (s)");

subplot(2,2,4);
title("States vs state estimate - velocities");
hold on
plot(t,x_hat_nol(3,:),'DisplayName','x-hat-theta_d');
plot(t,x_hat_nol(4,:),'DisplayName','x-hat-alpha_d');
plot(t,x_nol(3,:),'DisplayName','x-theta_d');
plot(t,x_nol(4,:),'DisplayName','x-alpha_d');
legend
ylabel("angle and velocity (rad and rad/s)")
xlabel("time (s)");




end









