function data = reduce_trip_length(data,trip_length)

    % Reduce trip length.
    last_imu_index = find(data.imu.time-data.imu.time(1)>trip_length,1,'first');
    data.imu.accelerometers = data.imu.accelerometers(:,1:last_imu_index);
    data.imu.gyroscopes = data.imu.gyroscopes(:,1:last_imu_index);
    data.imu.time = data.imu.time(1:last_imu_index);
    data.imu.index_vector = data.imu.index_vector(1:last_imu_index);        

    last_gnss_index = sum(data.imu.index_vector);
    data.gnss.speed = data.gnss.speed(1:last_gnss_index);
    data.gnss.course = data.gnss.course(1:last_gnss_index);
    data.gnss.velocity = data.gnss.velocity(:,1:last_gnss_index);        
    data.gnss.time = data.gnss.time(1:last_gnss_index);

end