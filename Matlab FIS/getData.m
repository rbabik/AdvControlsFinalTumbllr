% Function to get data from tables
function [data_T1_t, data_T1_dtheta, data_T1_theta,...
          data_T2_t, data_T2_dtheta, data_T2_theta,...
          data_T3_t, data_T3_dtheta, data_T3_theta,...
          data_T4_t, data_T4_dtheta, data_T4_theta,...
          data_T5_t, data_T5_dtheta, data_T5_theta] = getData(T)
    % Trial 1
    % Time
    data_T1_t = T.Trial_1_time;
    if (data_T1_t(1) ~= 0)
        data_T1_t = char(T.Trial_1_time);
        data_T1_t = str2num(data_T1_t(:,end-5:end));
        data_T1_t = data_T1_t(data_T1_t>0);
        data_T1_t = data_T1_t - data_T1_t(1);
        % Theta dot
        data_T1_dtheta = T.Trial_1_dtheta(1:size(data_T1_t,1),:);
        data_T1_dtheta = data_T1_dtheta - data_T1_dtheta(1);
        % Theta
        data_T1_theta = T.Trial_1_theta(1:size(data_T1_t,1),:);
        data_T1_theta = data_T1_theta - data_T1_theta(1);
    else
        data_T1_dtheta = zeros(size(data_T1_t,1),1);
        data_T1_theta = zeros(size(data_T1_t,1),1);
    end

    % Trial 2
    % Time
    data_T2_t = T.Trial_2_time;
    if (data_T2_t(1) ~= 0)
        data_T2_t = char(T.Trial_2_time);
        data_T2_t = str2num(data_T2_t(:,end-5:end));
        data_T2_t = data_T2_t(data_T2_t>0);
        data_T2_t = data_T2_t - data_T2_t(1);
        % Theta dot
        data_T2_dtheta = T.Trial_2_dtheta(1:size(data_T2_t,1),:);
        data_T2_dtheta = data_T2_dtheta - data_T2_dtheta(1);
        % Theta
        data_T2_theta = T.Trial_2_theta(1:size(data_T2_t,1),:);
        data_T2_theta = data_T2_theta - data_T2_theta(1);
    else
        data_T2_dtheta = zeros(size(data_T2_t,1),1);
        data_T2_theta = zeros(size(data_T2_t,1),1);
    end

    % Trial 3
    % Time
    data_T3_t = T.Trial_3_time;
    if (data_T3_t(1) ~= 0)
        data_T3_t = char(T.Trial_3_time);
        data_T3_t = str2num(data_T3_t(:,end-5:end));
        data_T3_t = data_T3_t(data_T3_t>0);
        data_T3_t = data_T3_t - data_T3_t(1);
        % Theta dot
        data_T3_dtheta = T.Trial_3_dtheta(1:size(data_T3_t,1),:);
        data_T3_dtheta = data_T3_dtheta - data_T3_dtheta(1);
        % Theta
        data_T3_theta = T.Trial_3_theta(1:size(data_T3_t,1),:);
        data_T3_theta = data_T3_theta - data_T3_theta(1);
    else
        data_T3_dtheta = zeros(size(data_T3_t,1),1);
        data_T3_theta = zeros(size(data_T3_t,1),1);
    end

    % Trial 4
    % Time
    data_T4_t = T.Trial_4_time;
    if (data_T4_t(1) ~= 0)
        data_T4_t = char(T.Trial_4_time);
        data_T4_t = str2num(data_T4_t(:,end-5:end));
        data_T4_t = data_T4_t(data_T4_t>0);
        data_T4_t = data_T4_t - data_T4_t(1);
        % Theta dot
        data_T4_dtheta = T.Trial_4_dtheta(1:size(data_T4_t,1),:);
        data_T4_dtheta = data_T4_dtheta - data_T4_dtheta(1);
        % Theta
        data_T4_theta = T.Trial_4_theta(1:size(data_T4_t,1),:);
        data_T4_theta = data_T4_theta - data_T4_theta(1);
    else
        data_T4_dtheta = zeros(size(data_T4_t,1),1);
        data_T4_theta = zeros(size(data_T4_t,1),1);
    end

    % Trial 5
    % Time
    data_T5_t = T.Trial_5_time;
    if (data_T5_t(1) ~= 0)
        data_T5_t = char(T.Trial_5_time);
        data_T5_t = str2num(data_T5_t(:,end-5:end));
        data_T5_t = data_T5_t(data_T5_t>0);
        data_T5_t = data_T5_t - data_T5_t(1);
        % Theta dot
        data_T5_dtheta = T.Trial_5_dtheta(1:size(data_T5_t,1),:);
        data_T5_dtheta = data_T5_dtheta - data_T5_dtheta(1);
        % Theta
        data_T5_theta = T.Trial_5_theta(1:size(data_T5_t,1),:);
        data_T5_theta = data_T5_theta - data_T5_theta(1);
    else
        data_T5_dtheta = zeros(size(data_T5_t,1),1);
        data_T5_theta = zeros(size(data_T5_t,1),1);
    end
end