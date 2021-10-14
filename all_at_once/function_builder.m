fileID = fopen('sys_matrices.m','w');

%part 1
function_part1 = 'function [A,B,C,D] = sys_matrices(';
fprintf(fileID,function_part1);

%arguments
for i=1:size(parameters_to_est,1)
    fprintf(fileID,parameters_to_est{i,1});
    fprintf(fileID,',');
end

%part 2
function_part2 = 'Ts) \n';
fprintf(fileID,function_part2);

%known values
for i=1:size(parameters_known,1)
    fprintf(fileID,parameters_known{i,1});
    fprintf(fileID,' = ');
    fprintf(fileID,'%4.4f ;\n', eval(parameters_known{i,1}));
    %fprintf(fileID,',');
end

function_part4 = 'A = [-d_p*(m_p*r_m^2+I_m)/(I_m*m_p*r_p^2) d_m*r_m/(I_m*r_p) -g*(m_p*r_m^2 + I_m)/(I_m*r_p) (r_m*s_m)/(I_m*r_p); \n r_m*d_p/(I_m*r_p)                    -d_m/I_m          (g*m_p*r_m)/I_m                 -s_m/I_m;\n eye(2)      zeros(2)]; \n  B = [-m_c*r_m/(I_m*r_p); \n     m_c/I_m;\n     zeros(2,1)]; \n    C = [zeros(2) eye(2)]; \n    D = [0;0]; \n end';
fprintf(fileID,function_part4);

fclose(fileID);

%{
function [A,B,C,D] = sys_matrices(PARAMETERS_TO_EST) %Ts is needed for greyest
    A = [-d_p*(m_p*r_m^2+I_m)/(I_m*m_p*r_p^2) d_m*r_m/(I_m*r_p) -g*(m_p*r_m^2 + I_m)/(I_m*r_p) (r_m*s_m)/(I_m*r_p); 
         r_m*d_p/(I_m*r_p)                    -d_m/I_m          (g*m_p*r_m)/I_m                 -s_m/I_m;
         eye(2)      zeros(2)];
    B = [-m_c*r_m/(I_m*r_p); 
         m_c/I_m;
         zeros(2,1)];
    C = [zeros(2) eye(2)];
    D = [0;0];
end

%}
