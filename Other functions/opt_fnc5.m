function fnc_value = opt_fnc5(euler_angles,closest_indices_reduced,zv_indices,y_gnss,sigma,u)

    % Compute relevant IMU metrics
    y_imu = [([1 0 0]*Rot_Mat_Fnc(euler_angles)*u(1:3,closest_indices_reduced))';
             ([0 0 1]*Rot_Mat_Fnc(euler_angles)*u(1:3,zv_indices))'];              

    % Compute difference between GNSS and IMU metrics
    y = y_gnss - y_imu;    

    % Compute function value
    fnc_value = sum(y.^2./sigma.^2);

end