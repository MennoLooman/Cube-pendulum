function u = MPC_controller(x0)
%input: state x(1) output: u(1)... u(Horizon_C)
%given parameter: Horizon_P Horizon_C Ad Bd Cd Dd P Q R alpha_bound
%calculate Horizon_P ahead, objective: 1\2 x(N+1)' P x(N+1) + ...
%                        sum k=1^N 1\2 x(k)' Q x(k) + 1/2 u(k)' R u(k)
% such that constraints....
% opti vector xu = [x(1) x(2) ... x(Horizon_P+1) u(1) u(2) ... u(Horizon_P)]'

%sys behaviour
beq = [x0; zeros(4*Horizon_P,1)]; 

%optie process
xu = quadprog(Objective_H,[],Aleq,bleq,Aeq,beq);

%results
u = xu(4*(Horizon_P+1)+1:4*(Horizon_P+1)+Horizon_C);