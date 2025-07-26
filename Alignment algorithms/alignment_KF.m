function euler_angles = alignment_KF(data)

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

    R = Rot_Mat_Fnc([rp  0])';
    phis = zeros(length(inds),1);
    Rs = zeros(length(inds),1);
    for j = 1:length(inds)

        ind = inds(j);

        start_index = max(closest_indices(ind) - 30, 1);
        end_index = min(closest_indices(ind) + 30, size(data.imu.accelerometers, 2));                 

        rot_accs = R*data.imu.accelerometers(:, start_index:end_index);
        phis(j) = -pi-atan2(mean(rot_accs(2,:)),mean(rot_accs(1,:)));
        Rs(j) = 2^2/mean(abs(rot_accs(1,:)));

    end

    %%%%%%%%%%%%%%%%%
    % Kalman filter %
    %%%%%%%%%%%%%%%%%

    x = phis(1);
    P = Rs(1);
    Q = 0^2;
    for i = 2:length(phis)
        P = P+Q;
        R = Rs(i);
        K = P/(P+R);
        x = x+K*(mod(phis(i)-x+pi,2*pi)-pi);
        P = (1-K)*P;
    end

    R = Rot_Mat_Fnc([rp x]);   

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate Euler angles %
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    euler_angles = zeros(1,3);
    euler_angles(1) = atan2(R(3,2), R(3,3)); % Roll
    euler_angles(2) = -asin(R(3,1));         % Pitch
    euler_angles(3) = atan2(R(2,1), R(1,1)); % Yaw
    
end