%% Run model 
main;
%linearization;

clearvars -except u y Ac Bc x_d
%% Prediction
y_hat = zeros(size(y));
x_hat = zeros(size(y,1),4);
gamma = sdpvar(5,1);
Ac = [0 gamma(1)*gamma(2) gamma(3) 0; 0 gamma(2) gamma(4) 0;eye(2) zeros(2)];
Bc = [gamma(1)*gamma(5); gamma(5); 0; 0];

%% Optimisation
objective = 0;
x_hat = zeros(1,4);
for k = 2:size(y,1)
%     x_hat(k,:) = (Ac * x_hat(k-1,:)' + Bc * u(k-1,2))';
%     y_hat(k,:) = ([zeros(2) eye(2)] * x_hat(k,:)')';
    x_hat = (Ac * x_hat' + Bc * u(k-1,2))';
    y_hat = ([zeros(2) eye(2)] * x_hat')';
    objective = objective + norm( y(k,:)' - y_hat' )^2;
end 

%% optimize
optimize([],objective);
objective_val = value(objective);
parameters = value(gamma);