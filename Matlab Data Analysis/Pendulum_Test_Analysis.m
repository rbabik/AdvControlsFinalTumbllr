%%
clc; clear all;

%%
% Read in data from Excel sheet
varNames = {'Trial_1_time_dtheta', 'Trial_1_theta', 'blank_1',...
    'Trial_2_time_dtheta', 'Trial_2_theta', 'blank_2',...
    'Trial_3_time_dtheta', 'Trial_3_theta', 'blank_3',...
    'Trial_4_time_dtheta', 'Trial_4_theta', 'blank_4',...
    'Trial_5_time_dtheta', 'Trial_5_theta'
};
opts_45 = spreadsheetImportOptions('Sheet', 2, 'VariableNames', varNames, 'DataRange', 'A3');
opts_60 = spreadsheetImportOptions('Sheet', 3, 'VariableNames', varNames, 'DataRange', 'A3');
opts_75 = spreadsheetImportOptions('Sheet', 4, 'VariableNames', varNames, 'DataRange', 'A3');
opts_90 = spreadsheetImportOptions('Sheet', 5, 'VariableNames', varNames, 'DataRange', 'A3');
data_45 = readtable('FIS_Trials_Pendulum.xlsx', opts_45);
data_60 = readtable('FIS_Trials_Pendulum.xlsx', opts_60);
data_75 = readtable('FIS_Trials_Pendulum.xlsx', opts_75);
data_90 = readtable('FIS_Trials_Pendulum.xlsx', opts_90);

% Remove blanks
for i = [12, 9, 6, 3]
    data_45(:,i) = [];
    data_60(:,i) = [];
    data_75(:,i) = [];
    data_90(:,i) = [];
end

% Write tables to a .txt file
test_type_45 = 'FIS_Pendulum_';
test_type_60 = 'FIS_Pendulum_';
test_type_75 = 'FIS_Pendulum_';
test_type_90 = 'FIS_Pendulum_';
filename_45 = strcat(test_type_45, 'data_45.txt');
filename_60 = strcat(test_type_60, 'data_60.txt');
filename_75 = strcat(test_type_75, 'data_75.txt');
filename_90 = strcat(test_type_90, 'data_90.txt');
writetable(data_45, filename_45,'Delimiter',' ','QuoteStrings',false)
writetable(data_60, filename_60,'Delimiter',' ','QuoteStrings',false)
writetable(data_75, filename_75,'Delimiter',' ','QuoteStrings',false)
writetable(data_90, filename_90,'Delimiter',' ','QuoteStrings',false)

%%
% Read the file back in
data_45 = readtable(filename_45, 'ReadRowNames', false);
data_60 = readtable(filename_60, 'ReadRowNames', false);
data_75 = readtable(filename_75, 'ReadRowNames', false);
data_90 = readtable(filename_90, 'ReadRowNames', false);

% Remove arrows
for j = [18, 14, 10, 6, 2]
    data_45(:,j) = [];
    data_60(:,j) = [];
    data_75(:,j) = [];
    data_90(:,j) = [];
end

% Assign proper variable names
[data_45.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta',...
    'Trial_4_time', 'Trial_4_dtheta', 'Trial_4_theta',...
    'Trial_5_time', 'Trial_5_dtheta', 'Trial_5_theta'
});
[data_60.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta',...
    'Trial_4_time', 'Trial_4_dtheta', 'Trial_4_theta',...
    'Trial_5_time', 'Trial_5_dtheta', 'Trial_5_theta'
});
[data_75.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta',...
    'Trial_4_time', 'Trial_4_dtheta', 'Trial_4_theta',...
    'Trial_5_time', 'Trial_5_dtheta', 'Trial_5_theta'
});
[data_90.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta',...
    'Trial_4_time', 'Trial_4_dtheta', 'Trial_4_theta',...
    'Trial_5_time', 'Trial_5_dtheta', 'Trial_5_theta'
});

%%
% Get data for 45 degrees
[data_45_T1_t, data_45_T1_dtheta, data_45_T1_theta,...
data_45_T2_t, data_45_T2_dtheta, data_45_T2_theta,...
data_45_T3_t, data_45_T3_dtheta, data_45_T3_theta,...
data_45_T4_t, data_45_T4_dtheta, data_45_T4_theta,...
data_45_T5_t, data_45_T5_dtheta, data_45_T5_theta] = getData(data_45);

% Get data for 60 degrees
[data_60_T1_t, data_60_T1_dtheta, data_60_T1_theta,...
data_60_T2_t, data_60_T2_dtheta, data_60_T2_theta,...
data_60_T3_t, data_60_T3_dtheta, data_60_T3_theta,...
data_60_T4_t, data_60_T4_dtheta, data_60_T4_theta,...
data_60_T5_t, data_60_T5_dtheta, data_60_T5_theta] = getData(data_60);

% Get data for 75 degrees
[data_75_T1_t, data_75_T1_dtheta, data_75_T1_theta,...
data_75_T2_t, data_75_T2_dtheta, data_75_T2_theta,...
data_75_T3_t, data_75_T3_dtheta, data_75_T3_theta,...
data_75_T4_t, data_75_T4_dtheta, data_75_T4_theta,...
data_75_T5_t, data_75_T5_dtheta, data_75_T5_theta] = getData(data_75);

% Get data for 90 degrees
[data_90_T1_t, data_90_T1_dtheta, data_90_T1_theta,...
data_90_T2_t, data_90_T2_dtheta, data_90_T2_theta,...
data_90_T3_t, data_90_T3_dtheta, data_90_T3_theta,...
data_90_T4_t, data_90_T4_dtheta, data_90_T4_theta,...
data_90_T5_t, data_90_T5_dtheta, data_90_T5_theta] = getData(data_90);

%%
% Results
% 45 degrees
[data_45_T1_max_error, data_45_T1_L2_norm_error] = getError(data_45_T1_t, data_45_T1_theta, 0);
[data_45_T2_max_error, data_45_T2_L2_norm_error] = getError(data_45_T2_t, data_45_T2_theta, 0);
[data_45_T3_max_error, data_45_T3_L2_norm_error] = getError(data_45_T3_t, data_45_T3_theta, 0);
[data_45_T4_max_error, data_45_T4_L2_norm_error] = getError(data_45_T4_t, data_45_T4_theta, 0);
[data_45_T5_max_error, data_45_T5_L2_norm_error] = getError(data_45_T5_t, data_45_T5_theta, 0);

% 60 degrees
[data_60_T1_max_error, data_60_T1_L2_norm_error] = getError(data_60_T1_t, data_60_T1_theta, 0);
[data_60_T2_max_error, data_60_T2_L2_norm_error] = getError(data_60_T2_t, data_60_T2_theta, 0);
[data_60_T3_max_error, data_60_T3_L2_norm_error] = getError(data_60_T3_t, data_60_T3_theta, 0);
[data_60_T4_max_error, data_60_T4_L2_norm_error] = getError(data_60_T4_t, data_60_T4_theta, 0);
[data_60_T5_max_error, data_60_T5_L2_norm_error] = getError(data_60_T5_t, data_60_T5_theta, 0);

% 75 degrees
[data_75_T1_max_error, data_75_T1_L2_norm_error] = getError(data_75_T1_t, data_75_T1_theta, 0);
[data_75_T2_max_error, data_75_T2_L2_norm_error] = getError(data_75_T2_t, data_75_T2_theta, 0);
[data_75_T3_max_error, data_75_T3_L2_norm_error] = getError(data_75_T3_t, data_75_T3_theta, 0);
[data_75_T4_max_error, data_75_T4_L2_norm_error] = getError(data_75_T4_t, data_75_T4_theta, 0);
[data_75_T5_max_error, data_75_T5_L2_norm_error] = getError(data_75_T5_t, data_75_T5_theta, 0);

% 90 degrees
[data_90_T1_max_error, data_90_T1_L2_norm_error] = getError(data_90_T1_t, data_90_T1_theta, 0);
[data_90_T2_max_error, data_90_T2_L2_norm_error] = getError(data_90_T2_t, data_90_T2_theta, 0);
[data_90_T3_max_error, data_90_T3_L2_norm_error] = getError(data_90_T3_t, data_90_T3_theta, 0);
[data_90_T4_max_error, data_90_T4_L2_norm_error] = getError(data_90_T4_t, data_90_T4_theta, 0);
[data_90_T5_max_error, data_90_T5_L2_norm_error] = getError(data_90_T5_t, data_90_T5_theta, 0);

% Write results to a text file
% Max error results
data_45_max_error = [data_45_T1_max_error; data_45_T2_max_error; data_45_T3_max_error; data_45_T4_max_error; data_45_T5_max_error];
data_60_max_error = [data_60_T1_max_error; data_60_T2_max_error; data_60_T3_max_error; data_60_T4_max_error; data_60_T5_max_error];
data_75_max_error = [data_75_T1_max_error; data_75_T2_max_error; data_75_T3_max_error; data_75_T4_max_error; data_75_T5_max_error];
data_90_max_error = [data_90_T1_max_error; data_90_T2_max_error; data_90_T3_max_error; data_90_T4_max_error; data_90_T5_max_error];

% L2 norm error
data_45_L2_norm_error = [data_45_T1_L2_norm_error; data_45_T2_L2_norm_error; data_45_T3_L2_norm_error; data_45_T4_L2_norm_error; data_45_T5_L2_norm_error];
data_60_L2_norm_error = [data_60_T1_L2_norm_error; data_60_T2_L2_norm_error; data_60_T3_L2_norm_error; data_60_T4_L2_norm_error; data_60_T5_L2_norm_error];
data_75_L2_norm_error = [data_75_T1_L2_norm_error; data_75_T2_L2_norm_error; data_75_T3_L2_norm_error; data_75_T4_L2_norm_error; data_75_T5_L2_norm_error];
data_90_L2_norm_error = [data_90_T1_L2_norm_error; data_90_T2_L2_norm_error; data_90_T3_L2_norm_error; data_90_T4_L2_norm_error; data_90_T5_L2_norm_error];

% Compile results
results = [data_45_max_error, data_45_L2_norm_error,...
    data_60_max_error, data_60_L2_norm_error,...
    data_75_max_error, data_75_L2_norm_error,...
    data_90_max_error, data_90_L2_norm_error];

% Write files to a .csv
results = cell2table(num2cell(results));
writetable(results, 'FIS_Pendulum_Results.csv');

%% Plot results
clear all;

figure('Name', 'Data 75 Trial 5')
plot(data_75_T5_t, data_75_T5_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Pendulum Drop Angle = 75 deg)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')
figure('Name', 'Data 75 Trial 4')
plot(data_75_T4_t, data_75_T4_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Pendulum Drop Angle = 75 deg)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')
figure('Name', 'Data 75 Trial 3')
plot(data_75_T3_t, data_75_T3_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Pendulum Drop Angle = 75 deg)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')
figure('Name', 'Data 75 Trial 2')
plot(data_75_T2_t, data_75_T2_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Pendulum Drop Angle = 75 deg)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')
figure('Name', 'Data 75 Trial 1')
plot(data_75_T1_t, data_75_T1_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Pendulum Drop Angle = 75 deg)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')