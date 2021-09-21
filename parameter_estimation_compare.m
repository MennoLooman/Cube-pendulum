%% Run first 
load('data/run5.mat');
t1 = u(:,1);
u1 = u(:,2);
y1 = y;
x1_hat = zeros(size(y1,1),4);
y1_hat = zeros(size(y1,1),2);
for i = 2:size(y1,1)
    %x1_hat(i,:) = x1_hat(i-1,:) + h * (A_est * x1_hat(i-1,:)' + B_est * u1(i-1))';
    x1_hat(i,:) = (sys_d.A * x1_hat(i-1,:)' + sys_d.B * u1(i-1))';
    y1_hat(i,:) = x1_hat(i,3:4);
end
%% Run second
load('data/run4.mat');
t2 = u(:,1);
u2 = u(:,2);
y2 = y;
x2_hat = zeros(size(y2,1),4);
y2_hat = zeros(size(y2,1),2);
for i = 2:size(y2,1)
    %x2_hat(i,:) = x2_hat(i-1,:) + h * (A_est * x2_hat(i-1,:)' + B_est * u2(i-1))';
    x2_hat(i,:) = (sys_d.A * x2_hat(i-1,:)' + sys_d.B * u2(i-1))';
    y2_hat(i,:) = x2_hat(i,3:4);
end
%% Plot run 1
figure();
plot(t1,u1, 'DisplayName', 'input u');
hold on;
plot(t1,y1(:,1), 'DisplayName', 'output y1');
plot(t1,y1(:,2), 'DisplayName', 'output y2');
plot(t1,y1_hat(:,1), 'DisplayName', 'estimate y1');
plot(t1,y1_hat(:,2), 'DisplayName', 'estimate y2');
hold off;
title('run used for estimation');
legend;
%% Plot run 2
figure();
plot(t2,u2, 'DisplayName', 'input u');
hold on;
plot(t2,y2(:,1), 'DisplayName', 'output y1');
plot(t2,y2(:,2), 'DisplayName', 'output y2');
plot(t2,y2_hat(:,1), 'DisplayName', 'estimate y1');
plot(t2,y2_hat(:,2), 'DisplayName', 'estimate y2');
hold off;
title('run used for validation');
legend;