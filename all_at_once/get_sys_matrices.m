%This is not 'nice' , but it works very well. ;)
%% make a script that uses the correct inputs for the function.
fileID = fopen('get_sys_matrices2.m','w');

%part 1
script_part1 = '[Ac, Bc,~,~] = sys_matrices(';
fprintf(fileID,script_part1);

%arguments
for script_iter=1:size(parameters_to_est,1)
    fprintf(fileID,'para_val(%d),',script_iter);
end

%part 2
function_part2 = 'Ts);';
fprintf(fileID,function_part2);

fclose(fileID);
%% save found parameters
para_val = sys_ge.Report.Parameters.ParVector;

%% 
get_sys_matrices2;