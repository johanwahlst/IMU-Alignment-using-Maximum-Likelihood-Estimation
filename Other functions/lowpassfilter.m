function data = lowpassfilter(data)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Low-pass filter inertial measurements %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Sampling frequency and cutoff frequency
    fs = 1 / median(diff(data.imu.time));  % Sampling frequency (adjust based on your data)
    fc = 0.5;  % Cutoff frequency (adjust as needed)

    % Apply low-pass filter to each column separately (X, Y, Z)
    data.imu.accelerometers = lowpass(data.imu.accelerometers', fc, fs)';  % Transpose, filter, then transpose back
    data.imu.gyroscopes = lowpass(data.imu.gyroscopes', fc, fs)';  % Transpose, filter, then transpose back

end