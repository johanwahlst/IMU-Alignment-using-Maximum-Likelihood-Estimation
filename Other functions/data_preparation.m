function data = data_preparation(filePath)

    table = readtable(filePath);

    data.imu.accelerometers = table2array(table(:,{'Var2','Var3','Var4'}))';
    data.imu.gyroscopes = table2array(table(:,{'Var5','Var6','Var7'}))';
    data.imu.time = table2array(table(:,{'Var1'}))';
    data.gnss.speed = table2array(table(:,'Var12'))';
    data.gnss.course = table2array(table(:,'Var8'))';
    data.gnss.velocity = table2array(table(:,{'Var9','Var10','Var11'}))';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Remove first ten seconds %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%

    ind_remove = 10*round(1/median(diff(data.imu.time)));
    data.imu.accelerometers = data.imu.accelerometers(:,ind_remove:end);
    data.imu.gyroscopes = data.imu.gyroscopes(:,ind_remove:end);
    data.imu.time = data.imu.time(ind_remove:end);
    
    data.gnss.speed = data.gnss.speed(ind_remove:end);
    data.gnss.course = data.gnss.course(ind_remove:end);
    data.gnss.velocity = data.gnss.velocity(:,ind_remove:end);

    %%%%%%%%%%%%%%%%%%%
    % Downsample GNSS %
    %%%%%%%%%%%%%%%%%%%

    data.gnss.time = data.imu.time(1:round(1/median(diff(data.imu.time))):end);
    data.gnss.speed = data.gnss.speed(1:round(1/median(diff(data.imu.time))):end);
    data.gnss.course = data.gnss.course(1:round(1/median(diff(data.imu.time))):end);
    data.gnss.velocity = data.gnss.velocity(:,1:round(1/median(diff(data.imu.time))):end);

    %%%%%%%%%%%%%%%%%%%
    % Downsample IMU %
    %%%%%%%%%%%%%%%%%%%

    % data.imu.accelerometers = data.imu.accelerometers(:,1:3:end);
    % data.imu.gyroscopes = data.imu.gyroscopes(:,1:3:end);
    % data.imu.time = data.imu.time(1:3:end);

    %%%%%%%%%%%%%%%%%%%%%%%
    % Create index vector %  
    %%%%%%%%%%%%%%%%%%%%%%%

    data.imu.index_vector = zeros(size(data.imu.time));
    data.imu.index_vector(1:round(1/median(diff(data.imu.time))):end) = 1;    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Time synchronization for differentiated GNSS measurements %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Finding matching time indices for GNSS and IMU
    my_gnss_utc = data.imu.time(data.imu.index_vector==1); 
    my_gnss_utc = my_gnss_utc(2:end);
    my_gnss_utc = my_gnss_utc-0.5;
    closest_indices = zeros(size(my_gnss_utc));
    for j = 1:length(my_gnss_utc)
        [~, closest_index] = min(abs(data.imu.time - my_gnss_utc(j)));
        closest_indices(j) = closest_index;
    end
    data.gnss.closest_indices = closest_indices;

end