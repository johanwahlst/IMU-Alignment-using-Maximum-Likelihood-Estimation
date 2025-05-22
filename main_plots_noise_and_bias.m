path(pathdef)
addpath(genpath(pwd))
clearvars

% Define the root folder
rootFolder = 'Data/2025_Mar';

% Get list of all files (including subfolders)
fileList = dir(fullfile(rootFolder, '**', '*')); 
fileList = fileList(~[fileList.isdir]); % Skip directories, process only files

N_Sim = 50;
Param_values = 20;
Param_fncs = 4;
N_symmetries = 24;

% Initialise matrix for results
Geodesic_distance = zeros(numel(fileList),N_symmetries,Param_values,Param_fncs,N_Sim,3);
Angles_distance = zeros(3,numel(fileList),N_symmetries,Param_values,Param_fncs,N_Sim,3);

% Iterate over each file
for i = 1:length(fileList)

    % Extract file path.
    filePath = fullfile(fileList(i).folder, fileList(i).name);

    % Prepare data.
    data = data_preparation(filePath);

    % Generate and apply artificial rotation.
    artificial_rotations = Octahedral_Group;

    for j = 1:N_symmetries % Iterate over artificial rotations
       
        disp([i,j])

        artificial_rotation = artificial_rotations(:,:,j);
        data_rotated = data;
        data_rotated.imu.accelerometers = artificial_rotation'*data.imu.accelerometers;
        data_rotated.imu.gyroscopes = artificial_rotation'*data.imu.gyroscopes;

        R = artificial_rotation;
        euler_angles(1) = atan2(R(3,2), R(3,3)); % Roll
        euler_angles(2) = -asin(R(3,1));         % Pitch
        euler_angles(3) = atan2(R(2,1), R(1,1)); % Yaw

        for k = 1:Param_values

           for l = 1:Param_fncs

                % Extract parameters
                params = parameters(k,l,Param_values);              

                for m = 1:N_Sim
                    
                    % Add measurement errors 
                    data_with_errors = add_errors(data_rotated,params);                
                  
                    % Run Nericell algorithm
                    euler_angles_nericell = alignment_Nericell(data_with_errors);
                    Geodesic_distance(i,j,k,l,m,1) = geodesic_distance(Rot_Mat_Fnc(euler_angles_nericell),artificial_rotation);   

                    % Run PCA-correlation
                    euler_angles_pca_correlation = alignment_Woo(data_with_errors);
                    Geodesic_distance(i,j,k,l,m,2) = geodesic_distance(Rot_Mat_Fnc(euler_angles_pca_correlation),artificial_rotation);

                    % Run ML estimation
                    euler_angles_ML = alignment_ML(data_with_errors,euler_angles_pca_correlation);
                    Geodesic_distance(i,j,k,l,m,3) = geodesic_distance(Rot_Mat_Fnc(euler_angles_ML),artificial_rotation);                 

                end

            end

        end

    end

    save(sprintf('Save_number_%d.mat', i)) 

end

save('simulations_noise_and_bias')

%%

load('simulations_noise_and_bias')
plot_noise_and_bias(Geodesic_distance,Param_values)