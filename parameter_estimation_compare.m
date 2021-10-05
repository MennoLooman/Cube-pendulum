%% Model 
[Ac, Bc,~,~] = sys_matrices2();
%Ac = sys_pem.A;
A = expm(Ac*h);
%B = inv(Ac)*(A-eye(4))*Bc;
B = h*Bc;

%% Run first estimation
load('data/AB/r5_AB_short2.mat');
t1 = u(:,1);
u1 = u(:,2);
y1 = y;
x0_1 = x0;
x1_hat = zeros(size(y1,1),size(Ac,1));
y1_hat = zeros(size(y1,1),2);
x1_hat(1,:) = x0_1;
y1_hat(1,:) = x0_1(3:4);
for i = 2:size(y1,1)
    x1_hat(i,:) = A * x1_hat(i-1,:)' + B * u1(i-1)';
    y1_hat(i,:) = x1_hat(i,end-size(Ac,1)/2+1:end);
end

%% Run second validation
load('data/AB/r1_AB_short.mat');
t2 = u(:,1);
u2 = u(:,2);
y2 = y;
x0_2 = x0;
x2_hat = zeros(size(y2,1),size(Ac,1));
x2_hat(1,:) = x0_2;
y2_hat = zeros(size(y2,1),2);
y2_hat(1,:) = x0_2(3:4);
for i = 2:size(y2,1)
    x2_hat(i,:) = A * x2_hat(i-1,:)' + B * u2(i-1)';
    y2_hat(i,:) = x2_hat(i,end-size(Ac,1)/2+1:end);
end

%% Plot run 1
clc
figure(1);
plot(t1,u1, 'DisplayName', 'input u');
hold on;
plot(t1,y1(:,1), 'DisplayName', 'theta y');
plot(t1,y1(:,2), 'DisplayName', 'alpha y');
plot(t1,y1_hat(:,1), 'DisplayName', 'theta y_{est}', 'LineWidth', 1);
plot(t1,y1_hat(:,2),'--','DisplayName', 'alpha y_est','LineWidth',1);
hold off;
title('run used for estimation');
legend;

fit_to_est1 = [(1-goodnessOfFit(y1_hat(:,1),y1(:,1),'NRMSE'))*100, (1-goodnessOfFit(y1_hat(:,2),y1(:,2),'NRMSE'))*100];
disp(fit_to_est1)

%% Plot run 2
figure(2);
plot(t2,u2, 'DisplayName', 'input u');
hold on;
plot(t2,y2(:,1), 'DisplayName', 'theta y');
plot(t2,y2(:,2), 'DisplayName', 'alpha y');
plot(t2,y2_hat(:,1), 'DisplayName', 'theta y_{est}','LineWidth', 1);
plot(t2,y2_hat(:,2),'--','DisplayName', 'alpha y_est','LineWidth', 1);
hold off;
title('run used for validation');
legend;


fit_to_est2 = [(1-goodnessOfFit(y2_hat(:,1),y2(:,1),'NRMSE'))*100, (1-goodnessOfFit(y2_hat(:,2),y2(:,2),'NRMSE'))*100];
disp(fit_to_est2)

function [A,B,C,D] = sys_matrices2() %Ts is needed for greyest
    r_p = 0.08063;%0.08101;
    d_p = 2.904e-05;%1.731e-05;
    m_p = 0.024;
    g = 9.813;%9.812;
    r_m = 0.0787;
    d_m = 4.2e-04;
    I_m = 1.5e-4;
    s_m = 2.6e-3;
    m_c = 0.2315;
    A = [-d_p*(m_p*r_m^2+I_m)/(I_m*m_p*r_p^2) d_m*r_m/(I_m*r_p) -g*(m_p*r_m^2 + I_m)/(I_m*r_p) (r_m*s_m)/(I_m*r_p); 
         r_m*d_p/(I_m*r_p)                    -d_m/I_m          (g*m_p*r_m)/I_m                 -s_m/I_m;
         eye(2)      zeros(2)];
    B = [-m_c*r_m/(I_m*r_p); 
         m_c/I_m;
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
end