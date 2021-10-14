clear,clc
%% run some needed scripts
%entire_sys_linearization;

%% Alter settings from here
% @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%% Choose runs
runs_to_analyze = {  '../data/AB_new/AB_new1_short.mat';
                     '../data/AB_new/AB_new2_short.mat';
                     '../data/AB_new/AB_new3_short.mat';
                     '../data/AB_new/AB_new4_short.mat';
                     '../data/AB_new/AB_new5_short.mat';
                     '../data/AB_new/AB_new6_short.mat';
                     '../data/AB_new/AB_new7_short.mat';
                     '../data/AB_new/AB_new8_short.mat';
                     '../data/AB_new/AB_new9_short.mat';};

%% choose parameters           
%give initial conditions. comment the variables that are not estimated.
r_m_I   = 0.0787;
d_m_I   = 4.2e-04;
I_m_I   = 1.5e-4;
m_c_I   = 0.2176;
s_m_I   = 2.6e-3;
%r_p_I  = 0.08101;
%m_p_I  = 0.024;
%d_p_I  = 1.731e-05;
%g_I    = 9.812;

%give values that are already known/estimated before, comment the variables that are known.
%r_m = 0.08415;
%d_m = 0.00046;
%I_m = 1.70e-4;
%m_c = 0.2176;
%s_m = 2.34e-3;
r_p = 0.0806;
m_p = 0.024;
d_p = 2.90e-05;
g = 9.812;

%% Do not alter from here
% @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%% make parameter vectors
variable_names = {'r_m','r_p','d_m','d_p','I_m','m_p','g','m_c','s_m'}; %dont change this, it's not for settings
variable_initials = {'r_m_I','r_p_I','d_m_I','d_p_I','I_m_I','m_p_I','g_I','m_c_I','s_m_I'}; %dont change this, it's not for settings

parameters_to_est = {1,2};
parameters_known = {1,2};

j = 1;
k = 1;
count = 0;
for i=1:length(variable_names)
    if exist(variable_initials{i})
        parameters_to_est(j+1,:) = {variable_names{i},eval(variable_initials{i})};
        j = j+1;
        count = count + 1;
    else
        parameters_known(k+1,:) = {variable_names{i},eval(variable_names{i})};
        k = k+1;
        count = count + 1;
    end
end

if count ~= length(variable_names)
    disp("error, not all variables where declared or initialized.");
    return
end
parameters_to_est   = parameters_to_est(2:end,:);
parameters_known    = parameters_known(2:end,:);

%% make space to print parameters
amount_of_runs = size(runs_to_analyze,1);
amount_of_est_pars = size(parameters_to_est,1);
amount_of_knowns = size(parameters_known,1);
%parameter table
runs = [1:1:amount_of_runs]';

l_vec = char(ones(size(runs,1),1)*124);
parameter_table = table(runs,l_vec,'VariableNames',{'runs','known'});
for i = 1:amount_of_knowns
    temp_table = table(runs,zeros(amount_of_runs,1),'VariableNames',{'runs',parameters_known{i,1}});
    parameter_table = join(parameter_table,temp_table);
end

parameter_table = join(parameter_table,table(runs,l_vec,'VariableNames',{'runs','Estimate'}));
for i = 1:amount_of_est_pars
    temp_table = table(runs,zeros(amount_of_runs,1),'VariableNames',{'runs',parameters_to_est{i,1}});
    parameter_table = join(parameter_table,temp_table);
end
    
%% make space to print validation scores

accuracy = table(runs,zeros(amount_of_runs,1),zeros(amount_of_runs,1),l_vec,'VariableNames',{'runs','estim t','estim a','|'});

var_type_array = {};
var_name_array = {};
for i = 1:amount_of_runs*2
    var_type_array{end+1} = 'double';
    if mod(i,2) == 0
        var_name_array{end+1} = ['r' num2str(i/2,'%d') 't'];
        var_name_array{end+1} = ['r' num2str(i/2,'%d') 'a'];
    end
end

temp_table = table('Size',[amount_of_runs,amount_of_runs*2], 'VariableTypes',var_type_array,'VariableNames',var_name_array);
accuracy = [accuracy,temp_table];
temp_table = table(char(ones(size(runs,1),1)*124),zeros(amount_of_runs,1),zeros(amount_of_runs,1),'VariableNames',{'-','ave val t','ave val a'});
accuracy = [accuracy,temp_table];

%% loop all runs
function_builder; %build function to evaluate, based on the variables to estimate.

for i = 1:amount_of_runs
%% estimation on one run
    load(runs_to_analyze{i});    %get data of run
    t = u(:,1);
    h = t(2)-t(1);
    u = u(:,2);
    data = iddata(y,u,h); % data file used for grey_est and pem
    
    % settings for estimator
    init_sys = idgrey(@sys_matrices,parameters_to_est,'c');
    opt_id = greyestOptions('InitialState',x0','Display','on','EnforceStability',true);
    % run estimator
    [sys_ge, x0_ge] = greyest(data,init_sys,opt_id);
    
    for j = 1:amount_of_knowns
        parameter_table(i,2+j) = parameters_known(j,2);
    end
    
    for j = 1:amount_of_est_pars
        parameter_table(i,3+size(parameters_known,1)+j) = {sys_ge.Structure.Parameters(1,j).Value};
    end
    
    accuracy{i,2:3} = sys_ge.Report.Fit.FitPercent';
    
%% validation on the other runs
    ave_val = zeros(1,2);
    for j = 1:amount_of_runs
       %get data to validate
        load(runs_to_analyze{j});
        t_v = u(:,1); 
        u_v = u(:,2);
        y_v = y;
        x0_v = [0,0,0,0];
       %get system matrices from estimation
        Ts = t(2)-t(1);
        get_sys_matrices;
        A = expm(Ac*h);
        B = h*Bc;
       %prepare some arrays
        x_v_hat = zeros(size(y_v,1),size(Ac,1));
        y_v_hat = zeros(size(y_v,1),2);
        x_v_hat(1,:) = x0_v;
        y_v_hat(1,:) = x0_v(3:4);
        
        for k = 2:size(y_v,1)
            x_v_hat(k,:) = A * x_v_hat(k-1,:)' + B * u_v(k-1)';
            y_v_hat(k,:) = x_v_hat(k,end-size(Ac,1)/2+1:end);
        end
        
        fit_to_est = [(1-goodnessOfFit(y_v_hat(:,1),y_v(:,1),'NRMSE'))*100, (1-goodnessOfFit(y_v_hat(:,2),y_v(:,2),'NRMSE'))*100];
        accuracy{i,3+j*2:4+j*2} = fit_to_est;
        if j ~= i
            ave_val = ave_val + fit_to_est;
        end
    end
    ave_val = ave_val/(amount_of_runs-1);
    accuracy{i,end-1:end} = ave_val;
end
%% Display Results
disp(parameter_table);
disp(accuracy);



%% estimate based on all the runs together (but keep one for validation)
%{
%make table
accuracy_all = table(1,0,0,1,'VariableNames',{'all e1','estim t','estim a','|'});

var_type_array = {};
var_name_array = {};
for i = 1:amount_of_runs*2
    var_type_array{end+1} = 'double';
    if mod(i,2) == 0
        var_name_array{end+1} = ['r' num2str(i/2,'%d') 't'];
        var_name_array{end+1} = ['r' num2str(i/2,'%d') 'a'];
    end
end

temp_table = table('Size',[1,amount_of_runs*2], 'VariableTypes',var_type_array,'VariableNames',var_name_array);
accuracy_all = [accuracy_all,temp_table];
temp_table = table(1,zeros(1,1),zeros(1,1),'VariableNames',{'-','ave val t','ave val a'});
accuracy_all = [accuracy_all,temp_table];

%% estimate

load(runs_to_analyze{1});    %get data of run
t = u(:,1);
h = t(2)-t(1);
u = u(:,2);
data_all = iddata(y,u,h); 

for i = 2:amount_of_runs-1
    load(runs_to_analyze{i});    %get data of run
    t = u(:,1);
    h = t(2)-t(1);
    u = u(:,2);
    data_i = iddata(y,u,h); % data file used for grey_est and pem
    data_all = merge(data_all,data_i);
end

[sys_ge, x0_ge] = greyest(data_all,init_sys,opt_id);

ave_val = zeros(1,2);
for j = 1:amount_of_runs
   %get data to validate
    load(runs_to_analyze{j});
    t_v = u(:,1); 
    u_v = u(:,2);
    y_v = y;
    x0_v = [0,0,0,0];
   %get system matrices from estimation
    Ts = t(2)-t(1);
    get_sys_matrices;
    A = expm(Ac*h);
    B = h*Bc;
   %prepare some arrays
    x_v_hat = zeros(size(y_v,1),size(Ac,1));
    y_v_hat = zeros(size(y_v,1),2);
    x_v_hat(1,:) = x0_v;
    y_v_hat(1,:) = x0_v(3:4);

    for k = 2:size(y_v,1)
        x_v_hat(k,:) = A * x_v_hat(k-1,:)' + B * u_v(k-1)';
        y_v_hat(k,:) = x_v_hat(k,end-size(Ac,1)/2+1:end);
    end

    fit_to_est = [(1-goodnessOfFit(y_v_hat(:,1),y_v(:,1),'NRMSE'))*100, (1-goodnessOfFit(y_v_hat(:,2),y_v(:,2),'NRMSE'))*100];
    accuracy_all{1,3+j*2:4+j*2} = fit_to_est;
    if j ~= 1
        ave_val = ave_val + fit_to_est;
    end
end
ave_val = ave_val/(amount_of_runs-1);
accuracy_all{1,end-1:end} = ave_val;






%}


















