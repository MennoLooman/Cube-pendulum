%% Model 
Ac = sys_pem.A;
A = expm(Ac*h);
%B = inv(sys_pem.A)*(Ac-eye(4))*sys_pem.B;
B = h*sys_pem.B;

%% Run first estimation
load('data/run8_pend.mat');
t1 = u(:,1);
u1 = u(:,2);
y1 = y;
x1_hat = zeros(size(y1,1),size(Ac,1));
y1_hat = zeros(size(y1,1),2);
x1_hat(1,:) = x0_ge';
y1_hat(1,:) = x0(2);
for i = 2:size(y1,1)
    x1_hat(i,:) = A * x1_hat(i-1,:)' + B * u1(i-1)';
    y1_hat(i,:) = x1_hat(i,end-size(Ac,1)/2+1:end);
end

%% Run second validation
load('data/run10_pend.mat');
t2 = u(:,1);
u2 = u(:,2);
y2 = y;
x2_hat = zeros(size(y2,1),size(Ac,1));
x2_hat(1,:) = x0;
y2_hat = zeros(size(y2,1),2);
y2_hat(1,:) = x0(2);
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
%plot(t1,y1(:,2), 'DisplayName', 'alpha y');
plot(t1,y1_hat(:,1), 'DisplayName', 'theta y_{est}', 'LineWidth', 1);
%plot(t1,y1_hat(:,2),'--','DisplayName', 'alpha y_est','LineWidth',1);
hold off;
title('run used for estimation');
legend;

fit_to_est1 = (1-goodnessOfFit(y1_hat(:,1),y1(:,1),'NRMSE'))*100;
disp(fit_to_est1)

%% Plot run 2
figure(2);
plot(t2,u2, 'DisplayName', 'input u');
hold on;
plot(t2,y2(:,1), 'DisplayName', 'theta y');
%plot(t2,y2(:,2), 'DisplayName', 'alpha y');
plot(t2,y2_hat(:,1), 'DisplayName', 'theta y_{est}','LineWidth', 1);
%plot(t2,y2_hat(:,2),'--','DisplayName', 'alpha y_est','LineWidth', 1);
hold off;
title('run used for validation');
legend;


fit_to_est2 = (1-goodnessOfFit(y2_hat(:,1),y2(:,1),'NRMSE'))*100;
disp(fit_to_est2)