function euler_angles = angle_distance(R1, R2)

    R = R1 * R2';
    euler_angles = zeros(3,1);
    euler_angles(1) = atan2(R(3,2), R(3,3)); % Roll
    euler_angles(2) = -asin(R(3,1));         % Pitch
    euler_angles(3) = atan2(R(2,1), R(1,1)); % Yaw

end