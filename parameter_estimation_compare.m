%% Model 
Ac = sys_pem.A;
A = expm(Ac*h);
%B = inv(sys_pem.A)*(Ac-eye(4))*sys_pem.B;
B = h*sys_pem.B;

%% Run first estimation
load('data/run5.mat');
t1 = u(:,1);
u1 = u(:,2);
y1 = y;
x1_hat = zeros(size(y1,1),4);
y1_hat = zeros(size(y1,1),2);
for i = 2:size(y1,1)
    x1_hat(i,:) = A * x1_hat(i-1,:)' + B * u1(i-1)';
    y1_hat(i,:) = x1_hat(i,3:4);
end
%% Run second validation
load('data/run4.mat');
t2 = u(:,1);
u2 = u(:,2);
y2 = y;
x2_hat = zeros(size(y2,1),4);
y2_hat = zeros(size(y2,1),2);
for i = 2:size(y2,1)
    x2_hat(i,:) = A * x2_hat(i-1,:)' + B * u2(i-1)';
    y2_hat(i,:) = x2_hat(i,3:4);
end
%% Plot run 1
figure();
plot(t1,u1, 'DisplayName', 'input u');
hold on;
plot(t1,y1(:,1), 'DisplayName', 'theta y');
plot(t1,y1(:,2), 'DisplayName', 'alpha y');
plot(t1,y1_hat(:,1), 'DisplayName', 'theta y_est', 'LineWidth', 2);
plot(t1,y1_hat(:,2), 'DisplayName', 'alpha y_est', 'LineWidth', 2);
hold off;
title('run used for estimation');
legend;
%% Plot run 2
figure();
plot(t2,u2, 'DisplayName', 'input u');
hold on;
plot(t2,y2(:,1), 'DisplayName', 'theta y');
plot(t2,y2(:,2), 'DisplayName', 'alpha y');
plot(t2,y2_hat(:,1), 'DisplayName', 'theta y_est','LineWidth', 2);
plot(t2,y2_hat(:,2), 'DisplayName', 'alpha y_est','LineWidth', 2);
hold off;
title('run used for validation');
legend;