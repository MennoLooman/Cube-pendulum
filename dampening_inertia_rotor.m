alpha1 = alpha(1:1/h,2);
alpha2 = alpha(1/h+1:end,2);
integral_alpha = alpha(end,2)-alpha(1,2);

Kr = 0.042; % Nm/A
R = 8.4; %ohm
V = amplitude*5;

torque_m = (Kr/R)*V;
integral_motor = torque_m*(1-0);

dampening_motor = integral_motor/integral_alpha;