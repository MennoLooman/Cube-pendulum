%% Run model 
%main;
%linearization;
load('data/run1.mat');
clearvars -except u y h

%% Prediction
y_hat = zeros(size(y));
x_hat = zeros(size(y,1),4);
%gamma = sdpvar(5,1);
 gamma = [2*0.085/0.129; sdpvar(4,1)];
Ac = [0 gamma(1)*gamma(2) gamma(3) 0; 0 gamma(2) gamma(4) 0;eye(2) zeros(2)];
Bc = [gamma(1)*gamma(5); gamma(5); 0; 0];

%% Quadratic programming