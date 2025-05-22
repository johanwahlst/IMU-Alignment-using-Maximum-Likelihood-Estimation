function euler_angles = alignment_Nericell(data)

    closest_indices = data.gnss.closest_indices;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute roll and pitch angles %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Find indices corresponding to the smallest variance
    zv_indices = identify_lowest_variance_instances(data.imu.gyroscopes, 5, round(size(data.imu.gyroscopes,2)/500));
    med_acc = median(data.imu.accelerometers(:,zv_indices),2);
    rp = [atan2(med_acc(2),med_acc(3)) atan2(-med_acc(1),sqrt(med_acc(2)^2+med_acc(3)^2))];

    %%%%%%%%%%%%%%%%%%%%%%
    % Compute yaw angles %
    %%%%%%%%%%%%%%%%%%%%%%

    diff_speed = diff(data.gnss.speed);
    diff_course = diff(unwrap(data.gnss.course));
    valid_indices = find(abs(diff_course) < 0.2);
    valid_speeds = diff_speed(valid_indices);
    [~, sorted_indices] = mink(valid_speeds, round(max(0.05*length(diff_speed),10)));    
    inds = valid_indices(sorted_indices);

    angle_diff = 0.04;

    % Rough optimization.
    xs = -pi:angle_diff:pi;
    ys = zeros(size(xs));
    for i = 1:length(xs)
        R = [1 0 0]*Rot_Mat_Fnc([rp  xs(i)])';
        % Get indices for all elements in `inds`
        start_indices = max(closest_indices(inds) - 30, 1);
        end_indices = min(closest_indices(inds) + 30, size(data.imu.accelerometers, 2));
              
        % Create the slices for all the indices in one go
        slices = arrayfun(@(start_idx, end_idx) data.imu.accelerometers(:, start_idx:end_idx), start_indices, end_indices, 'UniformOutput', false);

        % Concatenate all slices into one cell array and apply the sum
        ys(i) = ys(i) + sum(cellfun(@(slice) sum(R*slice), slices));
    end
    [~,i] = min(ys);
    x = xs(i);

    % Precise optimization.
    xs = (x-angle_diff):(2*angle_diff/100):(x+angle_diff);
    ys = zeros(size(xs));
    for i = 1:length(xs)
        R = [1 0 0]*Rot_Mat_Fnc([rp  xs(i)])';
        start_indices = max(closest_indices(inds) - 30, 1);
        end_indices = min(closest_indices(inds) + 30, size(data.imu.accelerometers, 2));

        % Create the slices for all the indices in one go
        slices = arrayfun(@(start_idx, end_idx) data.imu.accelerometers(:, start_idx:end_idx), start_indices, end_indices, 'UniformOutput', false);

        % Concatenate all slices into one cell array and apply the sum
        ys(i) = ys(i) + sum(cellfun(@(slice) sum(R*slice), slices));
    end

    [~,i] = min(ys);
    x = xs(i);

    R = Rot_Mat_Fnc([rp x]);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate Euler angles %
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    euler_angles = zeros(1,3);
    euler_angles(1) = atan2(R(3,2), R(3,3)); % Roll
    euler_angles(2) = -asin(R(3,1));         % Pitch
    euler_angles(3) = atan2(R(2,1), R(1,1)); % Yaw
    
end