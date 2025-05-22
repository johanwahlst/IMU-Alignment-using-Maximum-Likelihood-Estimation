function euler_angles_ML = alignment_ML(data,euler_angles_init)

    data = lowpassfilter(data);

    % Define the objective function to minimize
    [closest_indices_reduced,zv_indices,y_gnss,sigma,u] = opt_fnc5_pre(data);
    objective_fn = @(euler_angles) opt_fnc5(euler_angles,closest_indices_reduced,zv_indices,y_gnss,sigma,u);

    % Initial guess for the Euler angles (roll, pitch, yaw)
    initial_guess = euler_angles_init;
    
    % Define lower and upper bounds for Euler angles
    lb = [-2*pi, -pi, -2*pi];  % Lower bounds for Euler angles
    ub = [2*pi, pi, 2*pi];      % Upper bounds for Euler angles
    
    % Set options for fmincon (nonlinear constrained optimization)
    options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'interior-point', 'MaxIterations', 1000);     

    % Run the optimization to minimize the objective function
    euler_angles_ML = fmincon(objective_fn, initial_guess, [], [], [], [], lb, ub, [], options);

    % Finding matching time indices for GNSS and IMU.
    my_gnss_utc = data.imu.time(data.imu.index_vector==1); 
    my_gnss_utc = my_gnss_utc(2:end);
    my_gnss_utc = my_gnss_utc-0.5;
    closest_indices = zeros(size(my_gnss_utc));
    for j = 1:length(my_gnss_utc)
        [~, closest_index] = min(abs(data.imu.time - my_gnss_utc(j)));
        closest_indices(j) = closest_index;
    end

    % Accelerometer measurements in lateral axis
    acc = Rot_Mat_Fnc(euler_angles_ML)*data.imu.accelerometers;
    
    % Compute differentiated speed.
    speed_diff = diff(data.gnss.speed)./diff(data.gnss.time);

    corr1 = corr(acc(1,closest_indices)',speed_diff');
    %corr2 = corr(gyro(3,closest_indices(abs(data.gnss.speed(2:end))>5))', course_diff(abs(data.gnss.speed(2:end))>5)');
    corr2 = sum(acc(3,:));

    % Check ambiguity
    if(corr1<0 && corr2<0)
    
        rot_angles = [0 pi 0];
    
    elseif(corr1>0 && corr2<0)

        rot_angles = [pi 0 0];

    elseif(corr1<0 && corr2>0)

        rot_angles = [0 0 pi];
           
    end

    if(corr1<0 || corr2<0)

        R = Rot_Mat_Fnc(rot_angles)*Rot_Mat_Fnc(euler_angles_ML);    
        R = R';

        % Update Euler angles
        euler_angles_ML(1) = atan2(R(3,2), R(3,3)); % Roll
        euler_angles_ML(2) = -asin(R(3,1));         % Pitch
        euler_angles_ML(3) = atan2(R(2,1), R(1,1)); % Yaw

    end

end