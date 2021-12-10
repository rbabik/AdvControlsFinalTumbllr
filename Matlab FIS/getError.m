% Function to return maximum theta deviation and L2 norm error
function [max_error, L2_norm_error_normalized] = getError(data_t, data_theta, ref)
    % Get time difference for normalization
    time_diff = data_t(end) - data_t(1);
    if (time_diff < 0)
        time_diff = time_diff + 60;
    end
    % Initialize error values
    max_error = 0;
    L2_norm_error = 0;
    for i = 1:size(data_theta,1)
        % Update max_error if necessary
        if (abs(data_theta(i)) > max_error)
            max_error = abs(data_theta(i));
        end
        % Sum squared error values
        L2_norm_error = L2_norm_error + ((data_theta(i) - ref)^2);
    end
    % Take square root to yield L2 norm error
    L2_norm_error = sqrt(L2_norm_error);
    L2_norm_error_normalized = L2_norm_error/time_diff;
end