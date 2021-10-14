%main;
load('data/run5.mat');
h = 0.02;
clearvars -except u y h
data = iddata(y(:,2),u(:,2),h);
%% ARMAX
na = 4;
nb = 4; %or 4
nc = 4;
nk = 2; %?
[sys,ic] = armax(data, [na, nb, nc, nk]);
Ad = [-sys.A' [eye(size(sys.A,2)-1); zeros(1,size(sys.A,2)-1)]];
Bd = sys.B';