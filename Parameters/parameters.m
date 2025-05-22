function params = parameters(k,l,Param_values)

    params.accelerometer_noise = 0.05;
    params.gyroscope_noise = 2*pi/180; 
    params.accelerometer_bias = 0.05;
    params.gyroscope_bias = 2*pi/180;    

    if(l==1)
        acc_noise_low = 0.01;
        acc_noise_high = 0.5;        
        params.accelerometer_noise = exp(log(acc_noise_low):((log(acc_noise_high)-log(acc_noise_low))/(Param_values-1)):log(acc_noise_high));
        params.accelerometer_noise = params.accelerometer_noise(k);
    elseif(l==2)
        gyro_noise_low = 0.5*pi/180;
        gyro_noise_high = 15*pi/180;                
        params.gyroscope_noise = exp(log(gyro_noise_low):((log(gyro_noise_high)-log(gyro_noise_low))/(Param_values-1)):log(gyro_noise_high));
        params.gyroscope_noise = params.gyroscope_noise(k);
    elseif(l==3)
        acc_bias_low = 0.01;
        acc_bias_high = 0.5;        
        params.accelerometer_bias = exp(log(acc_bias_low):((log(acc_bias_high)-log(acc_bias_low))/(Param_values-1)):log(acc_bias_high));
        params.accelerometer_bias = params.accelerometer_bias(k);
    elseif(l==4)
        gyro_bias_low = 0.5*pi/180;
        gyro_bias_high = 15*pi/180;        
        params.gyroscope_bias = exp(log(gyro_bias_low):((log(gyro_bias_high)-log(gyro_bias_low))/(Param_values-1)):log(gyro_bias_high));
        params.gyroscope_bias = params.gyroscope_bias(k);
    end

    params.speed_noise = 0.2;
    params.course_noise = 10*pi/180;

end