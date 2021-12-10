%%
clc; clear all

%%
% Read in data from Excel sheet
varNames = {'Trial_1_time_dtheta', 'Trial_1_theta', 'blank_1',...
    'Trial_2_time_dtheta', 'Trial_2_theta', 'blank_2',...
    'Trial_3_time_dtheta', 'Trial_3_theta'
};
opts_10 = spreadsheetImportOptions('Sheet', 2, 'VariableNames', varNames, 'DataRange', 'A3');
opts_20 = spreadsheetImportOptions('Sheet', 3, 'VariableNames', varNames, 'DataRange', 'A3');
opts_30 = spreadsheetImportOptions('Sheet', 4, 'VariableNames', varNames, 'DataRange', 'A3');
opts_40 = spreadsheetImportOptions('Sheet', 5, 'VariableNames', varNames, 'DataRange', 'A3');
opts_50 = spreadsheetImportOptions('Sheet', 6, 'VariableNames', varNames, 'DataRange', 'A3');
opts_60 = spreadsheetImportOptions('Sheet', 7, 'VariableNames', varNames, 'DataRange', 'A3');
data_10 = readtable('FIS_Drop_Test_Nutz.xlsx', opts_10);
data_20 = readtable('FIS_Drop_Test_Nutz.xlsx', opts_20);
data_30 = readtable('FIS_Drop_Test_Nutz.xlsx', opts_30);
data_40 = readtable('FIS_Drop_Test_Nutz.xlsx', opts_40);
data_50 = readtable('FIS_Drop_Test_Nutz.xlsx', opts_50);
data_60 = readtable('FIS_Drop_Test_Nutz.xlsx', opts_60);

% Remove blanks
for i = [6, 3]
    data_10(:,i) = [];
    data_20(:,i) = [];
    data_30(:,i) = [];
    data_40(:,i) = [];
    data_50(:,i) = [];
    data_60(:,i) = [];
end

% Write tables to a .txt file
test_type_10 = 'FIS_Drop_';
test_type_20 = 'FIS_Drop_';
test_type_30 = 'FIS_Drop_';
test_type_40 = 'FIS_Drop_';
test_type_50 = 'FIS_Drop_';
test_type_60 = 'FIS_Drop_';
filename_10 = strcat(test_type_10, 'data_10.txt');
filename_20 = strcat(test_type_20, 'data_20.txt');
filename_30 = strcat(test_type_30, 'data_30.txt');
filename_40 = strcat(test_type_40, 'data_40.txt');
filename_50 = strcat(test_type_50, 'data_50.txt');
filename_60 = strcat(test_type_60, 'data_60.txt');
writetable(data_10, filename_10,'Delimiter',' ','QuoteStrings',false)
writetable(data_20, filename_20,'Delimiter',' ','QuoteStrings',false)
writetable(data_30, filename_30,'Delimiter',' ','QuoteStrings',false)
writetable(data_40, filename_40,'Delimiter',' ','QuoteStrings',false)
writetable(data_50, filename_50,'Delimiter',' ','QuoteStrings',false)
writetable(data_60, filename_60,'Delimiter',' ','QuoteStrings',false)

%%
% Read the file back in
data_10 = readtable(filename_10, 'ReadRowNames', false);
data_20 = readtable(filename_20, 'ReadRowNames', false);
data_30 = readtable(filename_30, 'ReadRowNames', false,'Delimiter',' ');
data_40 = readtable(filename_40, 'ReadRowNames', false);
data_50 = readtable(filename_50, 'ReadRowNames', false);
data_60 = readtable(filename_60, 'ReadRowNames', false);

% Remove arrows
for j = [10, 6, 2]
    data_10(:,j) = [];
    data_20(:,j) = [];
    data_30(:,j) = [];
    data_40(:,j) = [];
    data_50(:,j) = [];
    data_60(:,j) = [];
end

% Assign proper variable names
[data_10.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta'
});
[data_20.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta'
});
[data_30.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta'
});
[data_40.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta'
});
[data_50.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta'
});
[data_60.Properties.VariableNames] = deal({'Trial_1_time', 'Trial_1_dtheta', 'Trial_1_theta',...
    'Trial_2_time', 'Trial_2_dtheta', 'Trial_2_theta',...
    'Trial_3_time', 'Trial_3_dtheta', 'Trial_3_theta'
});

%%
% Get data for 10
[data_10_T1_t, data_10_T1_dtheta, data_10_T1_theta,...
data_10_T2_t, data_10_T2_dtheta, data_10_T2_theta,...
data_10_T3_t, data_10_T3_dtheta, data_10_T3_theta] = getData_Drop(data_10);

% Get data for 20
[data_20_T1_t, data_20_T1_dtheta, data_20_T1_theta,...
data_20_T2_t, data_20_T2_dtheta, data_20_T2_theta,...
data_20_T3_t, data_20_T3_dtheta, data_20_T3_theta] = getData_Drop(data_20);

% Get data for 30
[data_30_T1_t, data_30_T1_dtheta, data_30_T1_theta,...
data_30_T2_t, data_30_T2_dtheta, data_30_T2_theta,...
data_30_T3_t, data_30_T3_dtheta, data_30_T3_theta] = getData_Drop(data_30);

% Get data for 40
[data_40_T1_t, data_40_T1_dtheta, data_40_T1_theta,...
data_40_T2_t, data_40_T2_dtheta, data_40_T2_theta,...
data_40_T3_t, data_40_T3_dtheta, data_40_T3_theta] = getData_Drop(data_40);

% Get data for 50
[data_50_T1_t, data_50_T1_dtheta, data_50_T1_theta,...
data_50_T2_t, data_50_T2_dtheta, data_50_T2_theta,...
data_50_T3_t, data_50_T3_dtheta, data_50_T3_theta] = getData_Drop(data_50);

% Get data for 60
[data_60_T1_t, data_60_T1_dtheta, data_60_T1_theta,...
data_60_T2_t, data_60_T2_dtheta, data_60_T2_theta,...
data_60_T3_t, data_60_T3_dtheta, data_60_T3_theta] = getData_Drop(data_60);

%%
% Results
% 10
[data_10_T1_max_error, data_10_T1_L2_norm_error] = getError(data_10_T1_t, data_10_T1_theta, 0);
[data_10_T2_max_error, data_10_T2_L2_norm_error] = getError(data_10_T2_t, data_10_T2_theta, 0);
[data_10_T3_max_error, data_10_T3_L2_norm_error] = getError(data_10_T3_t, data_10_T3_theta, 0);

% 20
[data_20_T1_max_error, data_20_T1_L2_norm_error] = getError(data_20_T1_t, data_20_T1_theta, 0);
[data_20_T2_max_error, data_20_T2_L2_norm_error] = getError(data_20_T2_t, data_20_T2_theta, 0);
[data_20_T3_max_error, data_20_T3_L2_norm_error] = getError(data_20_T3_t, data_20_T3_theta, 0);

% 30
[data_30_T1_max_error, data_30_T1_L2_norm_error] = getError(data_30_T1_t, data_30_T1_theta, 0);
[data_30_T2_max_error, data_30_T2_L2_norm_error] = getError(data_30_T2_t, data_30_T2_theta, 0);
[data_30_T3_max_error, data_30_T3_L2_norm_error] = getError(data_30_T3_t, data_30_T3_theta, 0);

% 40
[data_40_T1_max_error, data_40_T1_L2_norm_error] = getError(data_40_T1_t, data_40_T1_theta, 0);
[data_40_T2_max_error, data_40_T2_L2_norm_error] = getError(data_40_T2_t, data_40_T2_theta, 0);
[data_40_T3_max_error, data_40_T3_L2_norm_error] = getError(data_40_T3_t, data_40_T3_theta, 0);

% 50
[data_50_T1_max_error, data_50_T1_L2_norm_error] = getError(data_50_T1_t, data_50_T1_theta, 0);
[data_50_T2_max_error, data_50_T2_L2_norm_error] = getError(data_50_T2_t, data_50_T2_theta, 0);
[data_50_T3_max_error, data_50_T3_L2_norm_error] = getError(data_50_T3_t, data_50_T3_theta, 0);

% 60
[data_60_T1_max_error, data_60_T1_L2_norm_error] = getError(data_60_T1_t, data_60_T1_theta, 0);
[data_60_T2_max_error, data_60_T2_L2_norm_error] = getError(data_60_T2_t, data_60_T2_theta, 0);
[data_60_T3_max_error, data_60_T3_L2_norm_error] = getError(data_60_T3_t, data_60_T3_theta, 0);

% Write results to a text file
% Max error results
data_10_max_error = [data_10_T1_max_error; data_10_T2_max_error; data_10_T3_max_error];
data_20_max_error = [data_20_T1_max_error; data_20_T2_max_error; data_20_T3_max_error];
data_30_max_error = [data_30_T1_max_error; data_30_T2_max_error; data_30_T3_max_error];
data_40_max_error = [data_40_T1_max_error; data_40_T2_max_error; data_40_T3_max_error];
data_50_max_error = [data_50_T1_max_error; data_50_T2_max_error; data_50_T3_max_error];
data_60_max_error = [data_60_T1_max_error; data_60_T2_max_error; data_60_T3_max_error];

% L2 norm error
data_10_L2_norm_error = [data_10_T1_L2_norm_error; data_10_T2_L2_norm_error; data_10_T3_L2_norm_error];
data_20_L2_norm_error = [data_20_T1_L2_norm_error; data_20_T2_L2_norm_error; data_20_T3_L2_norm_error];
data_30_L2_norm_error = [data_30_T1_L2_norm_error; data_30_T2_L2_norm_error; data_30_T3_L2_norm_error];
data_40_L2_norm_error = [data_40_T1_L2_norm_error; data_40_T2_L2_norm_error; data_40_T3_L2_norm_error];
data_50_L2_norm_error = [data_50_T1_L2_norm_error; data_50_T2_L2_norm_error; data_50_T3_L2_norm_error];
data_60_L2_norm_error = [data_60_T1_L2_norm_error; data_60_T2_L2_norm_error; data_60_T3_L2_norm_error];

% Compile results
results = [data_10_max_error, data_10_L2_norm_error,...
    data_20_max_error, data_20_L2_norm_error,...
    data_30_max_error, data_30_L2_norm_error,...
    data_40_max_error, data_40_L2_norm_error,...
    data_50_max_error, data_50_L2_norm_error,...
    data_60_max_error, data_60_L2_norm_error];

% Write files to a .csv
results = cell2table(num2cell(results));
writetable(results, 'FIS_Drop_Test_Results.csv');

%% Plot results
close all;

figure('Name', 'Data 10 Trial 3')
plot(data_10_T3_t, data_10_T3_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Mass Drop = 10 nuts)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')
xlim([0 5])
figure('Name', 'Data 10 Trial 2')
plot(data_10_T2_t, data_10_T2_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Mass Drop = 10 nuts)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')
figure('Name', 'Data 10 Trial 1')
plot(data_10_T1_t, data_10_T1_theta)
title('Plot of Robot Angle (deg) vs Time (s) (Mass Drop = 10 nuts)')
xlabel('Time (s)')
ylabel('Robot Angle (deg)')