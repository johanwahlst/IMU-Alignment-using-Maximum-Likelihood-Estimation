path(pathdef)
addpath(genpath(pwd))
clearvars

% Define the root folder
rootFolder = 'Data/2025_Mar';

% Get list of all files (including subfolders)
fileList = dir(fullfile(rootFolder, '**', '*')); 
fileList = fileList(~[fileList.isdir]); % Skip directories, process only files

N_Sim = 50;
N_symmetries = 24;
trip_lengths = 299:-269/19:30;

% Initialise matrix for results
Geodesic_distance = zeros(numel(fileList),N_symmetries,length(trip_lengths),N_Sim,3);
Angles_distance = zeros(3,numel(fileList),N_symmetries,length(trip_lengths),N_Sim,3);

% Iterate over each file
for i = 1:length(fileList)

    % Extract file path.
    filePath = fullfile(fileList(i).folder, fileList(i).name);
    
    % Prepare data.
    data = data_preparation(filePath);           

    % Generate artificial rotation.
    artificial_rotations = Octahedral_Group;

    for j = 1:N_symmetries % Iterate over artificial rotations

        disp([i,j])

        artificial_rotation = artificial_rotations(:,:,j);
        data_rotated = data;
        data_rotated.imu.accelerometers = artificial_rotation'*data.imu.accelerometers;
        data_rotated.imu.gyroscopes = artificial_rotation'*data.imu.gyroscopes;

        % Extract parameters
        params = parameters(0,0,5);

        for k = 1:length(trip_lengths)

            data_rotated = reduce_trip_length(data_rotated,trip_lengths(k));

            for m = 1:N_Sim               

                % Add measurement errors 
                data_with_errors = add_errors(data_rotated,params);

                % Run Nericell algorithm
                euler_angles_nericell = alignment_Nericell(data_with_errors);
                Geodesic_distance(i,j,k,m,1) = geodesic_distance(Rot_Mat_Fnc(euler_angles_nericell),artificial_rotation);
                Angles_distance(:,i,j,k,m,1) = angle_distance(Rot_Mat_Fnc(euler_angles_nericell),artificial_rotation);        

                % Run PCA-correlation
                euler_angles_pca_correlation = alignment_Woo(data_with_errors);
                Geodesic_distance(i,j,k,m,2) = geodesic_distance(Rot_Mat_Fnc(euler_angles_pca_correlation),artificial_rotation);
                Angles_distance(:,i,j,k,m,2) = angle_distance(Rot_Mat_Fnc(euler_angles_pca_correlation),artificial_rotation);

                % Run ML estimation
                euler_angles_ML = alignment_ML(data_with_errors,euler_angles_pca_correlation);
                Geodesic_distance(i,j,k,m,3) = geodesic_distance(Rot_Mat_Fnc(euler_angles_ML),artificial_rotation);                  
                Angles_distance(:,i,j,k,m,3) = angle_distance(Rot_Mat_Fnc(euler_angles_ML),artificial_rotation);                  
        
            end

        end

    end

    save(sprintf('Save_number_%d.mat', i)) 

end

save('simulations_trip_length')

%%

load('simulations_trip_length')
plot_trip_length(Geodesic_distance,trip_lengths)

%%

load('simulations_trip_length')
plot_angle_distances(Angles_distance,trip_lengths)