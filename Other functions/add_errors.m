function data = add_errors(data,params)

    % Add IMU biases.
    data.imu.accelerometers = data.imu.accelerometers + normrnd(0,params.accelerometer_bias,[3 1])*ones(1,size(data.imu.accelerometers,2));
    data.imu.gyroscopes = data.imu.gyroscopes + normrnd(0,params.gyroscope_bias,[3 1])*ones(1,size(data.imu.gyroscopes,2));

    % Add IMU noise.
    data.imu.accelerometers = data.imu.accelerometers + normrnd(0,params.accelerometer_noise,size(data.imu.accelerometers));
    data.imu.gyroscopes = data.imu.gyroscopes + normrnd(0,params.gyroscope_noise,size(data.imu.gyroscopes));

    % Add GNSS noise.
    data.gnss.speed = data.gnss.speed + normrnd(0,params.speed_noise,size(data.gnss.speed));
    data.gnss.course = data.gnss.course + normrnd(0,params.course_noise./max(data.gnss.speed,0.5));    
    data.gnss.vert_position_diff  = data.gnss.velocity(3,1:end-1)+normrnd(0,0.02,size(data.gnss.velocity(3,1:end-1)));

end