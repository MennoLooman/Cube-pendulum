%% Model 
r_p = 0.08063;%0.08101;
d_p = 2.904e-05;%1.731e-05;
m_p = 0.024;
g = 9.813;%9.812;
Ac = [-d_p/(m_p*r_p^2) -g/r_p; 1  0];
A = expm(Ac*h);
%B = inv(sys_pem.A)*(Ac-eye(4))*sys_pem.B;
B = h*sys_pem.B;

%% Run first estimation
load('data/run9_pend.mat');
t1 = u(:,1);
u1 = u(:,2);
y1 = y(:,1);
x1_hat = zeros(size(y1,1),2);
y1_hat = zeros(size(y1,1),1);
%x1_hat(round(locs1(1)/h),:) = [0,pks1(1)];
x1_hat(1,:) = x0_ge';
for i = 2:size(y1,1)
    x1_hat(i,:) = A * x1_hat(i-1,:)' + B * u1(i-1)';
    y1_hat(i,:) = x1_hat(i,2);
end
fit_to_est1 = (1-goodnessOfFit(y1_hat(:,1),y1(:,1),'NRMSE'))*100;
disp(fit_to_est1)
%% Run second validation
load('data/run10_pend.mat');
t2 = u(:,1);
u2 = u(:,2);
y2 = y(:,1);
[pks2,locs2] = findpeaks(y2,t2);
x2_hat = zeros(size(y2,1),2);
y2_hat = zeros(size(y2,1),1);
x2_hat(1,:) = x0';
for i = 2:size(y2,1)
    x2_hat(i,:) = A * x2_hat(i-1,:)' + B * u2(i-1)';
    y2_hat(i,:) = x2_hat(i,2);
end

fit_to_est2 = (1-goodnessOfFit(y2_hat(:,1),y2(:,1),'NRMSE'))*100;
disp(fit_to_est2)

%% Plot run 1
figure(1);
plot(t1,u1, 'DisplayName', 'input u');
hold on;
plot(t1,y1, 'DisplayName', 'theta y');
plot(t1,y1_hat, 'DisplayName', 'theta y_est', 'LineWidth', 2);
hold off;
title(append('run used for estimation, FTE:',num2str(fit_to_est2)));
legend;
%% Plot run 2
figure(2);
plot(t2,u2, 'DisplayName', 'input u');
hold on;
plot(t2,y2, 'DisplayName', 'theta y');
plot(t2,y2_hat, 'DisplayName', 'theta y_est','LineWidth', 2);
hold off;
title(append('run used for validation, FTE:',num2str(fit_to_est2)));
legend;