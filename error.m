clear
close all
clc

file_names = {'hierarchical_data1.mat', 'passivity_data1.mat', 'mpc_data1.mat','nlmpc_data1.mat'};
labels     = {'Hierarchical', 'Passivity', 'MPC','Non Linear MPC'};

n_file = numel(file_names);

max_vals.pos     = zeros(1, n_file);
max_vals.ang     = zeros(1, n_file);
max_vals.lin_vel = zeros(1, n_file);
max_vals.ang_vel = zeros(1, n_file);

for i = 1:n_file
    data = load(file_names{i});
    
    max_vals.pos(i)     = max(vecnorm(data.norm_pos.signals.values, 2, 2));
    max_vals.ang(i)     = max(vecnorm(data.norm_ang.signals.values, 2, 2));
    max_vals.lin_vel(i) = max(vecnorm(data.norm_lin_vel.signals.values, 2, 2));
    max_vals.ang_vel(i) = max(vecnorm(data.norm_ang_vel.signals.values, 2, 2));
end

fprintf('=== Confronto dei Valori Massimi ===\n\n');

for i = 1:n_file
    fprintf('Controller: %s\n', labels{i});
    fprintf('  -> Max Errore Posizione       = %.4e m\n', max_vals.pos(i));
    fprintf('  -> Max Errore Angolare        = %.4e rad\n', max_vals.ang(i));
    fprintf('  -> Max Errore Velocità Lineare= %.4e m/s\n', max_vals.lin_vel(i));
    fprintf('  -> Max Errore Velocità Angolare= %.4e rad/s\n\n', max_vals.ang_vel(i));
end