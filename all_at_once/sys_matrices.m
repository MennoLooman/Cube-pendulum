function [A,B,C,D] = sys_matrices(r_m,d_m,I_m,m_c,s_m,Ts) 
r_p = 0.0806 ;
d_p = 0.0000 ;
m_p = 0.0240 ;
g = 9.8120 ;
A = [-d_p*(m_p*r_m^2+I_m)/(I_m*m_p*r_p^2) d_m*r_m/(I_m*r_p) -g*(m_p*r_m^2 + I_m)/(I_m*r_p) (r_m*s_m)/(I_m*r_p); 
 r_m*d_p/(I_m*r_p)                    -d_m/I_m          (g*m_p*r_m)/I_m                 -s_m/I_m;
 eye(2)      zeros(2)]; 
  B = [-m_c*r_m/(I_m*r_p); 
     m_c/I_m;
     zeros(2,1)]; 
    C = [zeros(2) eye(2)]; 
    D = [0;0]; 
 end