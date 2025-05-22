function [closest_indices_reduced,zv_indices,y_gnss,sigma,u] = opt_fnc5_pre(data)
    
    % Define IMU measurements
    u = [data.imu.accelerometers; data.imu.gyroscopes];

    % Compute differentiated speed and course
    speed_diff = diff(data.gnss.speed)./diff(data.gnss.time);

    % Find sampling instances at which to apply lognidutinal acceleration model
    cond = abs([diff(speed_diff) 0])<0.5;        
    closest_indices_reduced = data.gnss.closest_indices(cond);
    
    zv_indices = identify_lowest_variance_instances(data.imu.gyroscopes, 5, round(size(data.imu.gyroscopes,2)/500));    

    g = sqrt(sum(median(data.imu.accelerometers').^2));
    
    % Compute relevant GNSS metrics
    y_gnss = [speed_diff(cond)'; g*ones(length(zv_indices),1)];

    % Define noise parameters
    sigma_v2 = 0.4*0.5;    
    sigma_gravity = 0.05;

    % Define noise vector
    sigma = [sigma_v2*ones(length(closest_indices_reduced),1); 
             sigma_gravity*ones(length(zv_indices),1)];

end