function smallest_var_indices = identify_lowest_variance_instances(gyroscopes, W, M)
    % Identifies the M sampling instances with the smallest variance.
    %
    % Parameters:
    % gyroscopes: 3xN matrix of gyroscope measurements.
    % W: Window size for variance computation.
    %
    % Returns:
    % smallest_var_indices: 1x250 vector containing the indices of the 250 lowest variance instances.
    
    N = size(gyroscopes, 2); % Number of samples
    variance_values = zeros(1, N); % Initialize variance storage
    
    half_W = floor(W / 2); % Half window size
    
    for k = 1:N
        % Define window range
        start_idx = max(1, k - half_W);
        end_idx = min(N, k + half_W);
        
        % Compute variance over the window
        var_gyro = var(gyroscopes(:, start_idx:end_idx), 0, 2);
        
        % Store the sum of variances as a metric
        variance_values(k) = sum(var_gyro);
    end
    
    % Identify the M indices with the smallest variance values
    [~, sorted_indices] = sort(variance_values);
    smallest_var_indices = sorted_indices(1:M);
end