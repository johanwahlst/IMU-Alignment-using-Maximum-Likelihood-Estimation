function euler_angles = alignment_Woo(data)

    % Finding matching time indices for GNSS and IMU.
    my_gnss_utc = data.imu.time(data.imu.index_vector==1); 
    my_gnss_utc = my_gnss_utc(2:end);
    my_gnss_utc = my_gnss_utc-0.5;
    closest_indices = zeros(size(my_gnss_utc));
    for j = 1:length(my_gnss_utc)
        [~, closest_index] = min(abs(data.imu.time - my_gnss_utc(j)));
        closest_indices(j) = closest_index;
    end

    % Compute differentiated speed and course.
    speed_diff = diff(data.gnss.speed)./diff(data.gnss.time);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find vertical vehicle axis %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Perform PCA on the gyroscope data to get the principal gyroscope axis
    coeffGyro = pca(data.imu.gyroscopes');
    principalGyroAxis = coeffGyro(:,1);
    
    % Vertical gyroscope measurements
    z_acc = principalGyroAxis'*data.imu.accelerometers;    

    % Check ambiguity
    if(sum(z_acc)<0)
        principalGyroAxis = -principalGyroAxis;
    end

    % Ensure principalGyroAxis is a unit vector
    principalGyroAxis = principalGyroAxis / norm(principalGyroAxis);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find longitudinal vehicle axis %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Pick an arbitrary vector not parallel to principalGyroAxis
    arbitrary_vec = [1; 0; 0];
    if abs(dot(arbitrary_vec, principalGyroAxis)) > 0.99  % If too aligned, pick another
        arbitrary_vec = [0; 1; 0];
    end
    
    % Compute first basis vector in the plane (orthogonal projection)
    u1 = arbitrary_vec - dot(arbitrary_vec, principalGyroAxis) * principalGyroAxis;
    u1 = u1 / norm(u1);  % Normalize to unit length
    
    u2 = cross(principalGyroAxis, u1);
    u2 = u2 / norm(u2);
    
    % Define objective function for optimization
    objective = @(theta) -corr(((cos(theta) * u1 + sin(theta) * u2)' * data.imu.accelerometers(:,closest_indices))', speed_diff');
    
    % Use fminbnd to find optimal theta in the range [0, 2*pi]
    optimal_theta = fminbnd(objective, 0, 2*pi);
    
    % Compute best axis using optimized theta
    best_axis = cos(optimal_theta) * u1 + sin(optimal_theta) * u2;

    % Accelerometer measurements in lateral axis
    x_acc = best_axis'*data.imu.accelerometers;
    
    % Check ambiguity
    if(corr(x_acc(closest_indices)',speed_diff')<0)
        best_axis = -best_axis;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find lateral vehicle axis %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    principalAccAxisInPlane = best_axis;

    % Define the vehicle frame axes
    z_vehicle = principalGyroAxis; % Vertical axis
    x_vehicle = principalAccAxisInPlane; % Longitudinal axis
    
    % Ensure orthogonality by computing the lateral axis
    y_vehicle = cross(z_vehicle, x_vehicle);
    
    % Normalize the axes to form an orthonormal basis
    z_vehicle = z_vehicle / norm(z_vehicle);
    y_vehicle = y_vehicle / norm(y_vehicle);
    x_vehicle = cross(y_vehicle, z_vehicle); % Ensure orthonormality
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute rotation matrix %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Construct the rotation matrix
    R = [x_vehicle, y_vehicle, z_vehicle];

    % Calculate Euler angles
    euler_angles = zeros(1,3);
    euler_angles(1) = atan2(R(3,2), R(3,3)); % Roll
    euler_angles(2) = -asin(R(3,1));         % Pitch
    euler_angles(3) = atan2(R(2,1), R(1,1)); % Yaw
    
end