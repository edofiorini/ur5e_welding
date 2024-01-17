%%%%% Matlab script for plotting and saving data %%%%%
clear all; close all; clc;

% import data
robot_pointer_csv_name = 'robot_pointer.csv';
ref_pointer_csv_name = 'ref_pointer.csv';
vertices_pointer_csv_name = 'vertices_pointer.csv';


robot_complete_csv_name = 'robot_complete.csv'; 
ref_complete_csv_name = 'ref_complete_mix.csv';
vertices_complete_csv_name = 'vertices_complete_mix.csv';

manual_name = 'robot_manual.csv';
robot_manual_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', manual_name);

% Specify the path to your CSV file
robot_pointer_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', robot_pointer_csv_name);
ref_pointer_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', ref_pointer_csv_name);
vert_pointer_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', vertices_pointer_csv_name);


robot_complete_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', robot_complete_csv_name);
ref_complete_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', ref_complete_csv_name);
vert_complete_csvFilePath = strcat('/home/saras/Workspace/ur5e_welding/src/smooth_trajectory/src/csv/', vertices_complete_csv_name);

% Read the CSV file
robot_pointer_data = csvread(robot_pointer_csvFilePath, 1, 0); % Assuming the first row contains headers
ref_pointer_data = csvread(ref_pointer_csvFilePath, 1, 0);
vert_pointer_data = csvread(vert_pointer_csvFilePath, 1, 0);

robot_complete_data = csvread(robot_complete_csvFilePath, 1, 0); % Assuming the first row contains headers
ref_complete_data = csvread(ref_complete_csvFilePath, 1, 0);
vert_complete_data = csvread(vert_complete_csvFilePath, 1, 0);

% manual data
manual_data = csvread(robot_manual_csvFilePath, 1, 0);
manual_x = manual_data(:, 2);
manual_y = manual_data(:, 3);

% Extract x, y, and z values
robot_pointer_x = robot_pointer_data(:, 2);
robot_pointer_y = robot_pointer_data(:, 3);
robot_pointer_z = robot_pointer_data(:, 4);

% delete begin
robot_pointer_x = robot_pointer_x(1100:end,:);
robot_pointer_y = robot_pointer_y(1100:end,:);
robot_pointer_z = robot_pointer_z(1100:end,:);

robot_complete_x = robot_complete_data(:, 2);
robot_complete_y = robot_complete_data(:, 3);
robot_rec_z = robot_complete_data(:, 4);

% Extract x, y, and z values
ref_pointer_x = ref_pointer_data(:, 1);
ref_pointer_y = ref_pointer_data(:, 2);
ref_z = ref_pointer_data(:, 3);

% delete begin
ref_pointer_x = ref_pointer_x(1100:end,:);
ref_pointer_y = ref_pointer_y(1100:end,:);
ref_z = ref_z(20:end,:);

ref_complete_x = ref_complete_data(:, 1);
ref_complete_y = ref_complete_data(:, 2);
ref_rec_z = ref_complete_data(:, 3);

% Extract x, y, and z values
vert_pointer_x = vert_pointer_data(:, 1);
vert_pointer_y = vert_pointer_data(:, 2);
vert_z = vert_pointer_data(:, 3);
vert_pointer_x(3,1) = vert_pointer_x(3,1) + 0.0049;
vert_pointer_x(6,1) = vert_pointer_x(6,1) - 0.002899;

vert_complete_x =  vert_complete_data(:, 1);
vert_complete_y = vert_complete_data(:, 2);
vert_rec_z = vert_complete_data(:, 3);
vert_complete_x(4,1) = vert_complete_x(4,1) + 0.0055;
vert_complete_x(7,1) = vert_complete_x(7,1) - 0.0075;


% color definition
LineWidth_size = 1;
robot_pointer = '#9036F5';
ref_pointer = '#F53684';
vert_pointer = '#F54036';

robot_complete = '#F5B128';
ref_complete = '#F5E016';
vert_complete = '#DEF516';

screensize = get( 0, 'Screensize');
figure('units','normalized','outerposition',[0 0 screensize(end)/screensize(end-1) 1]);

% pointer method
plot(robot_pointer_x, robot_pointer_y, '.', 'Color', robot_pointer, 'Linewidth', LineWidth_size);
hold on;
plot(ref_pointer_x, ref_pointer_y, '.', 'Color', ref_pointer, 'Linewidth', LineWidth_size);
hold on;
scatter(vert_pointer_x, vert_pointer_y,'.', 'Color', vert_pointer, 'Linewidth', LineWidth_size);

% complete line method
plot(robot_complete_x, robot_complete_y, '.', 'Color', robot_complete, 'Linewidth', LineWidth_size);
hold on;
plot(ref_complete_x, ref_complete_y, '.', 'Color', ref_complete, 'Linewidth', LineWidth_size);
hold on;
scatter(vert_complete_x, vert_complete_y,'.', 'Color', vert_complete, 'Linewidth', LineWidth_size);
hold on;

% manual 
scatter(manual_x, manual_y, '.', 'Color', 'green', 'Linewidth', LineWidth_size);

hold on;
axis equal
xlabel('X');
ylabel('Y');
legend('Robot Pose clothoids', 'Reference Pose clothoids', 'Vertices clothoids', 'Robot Pose complete line', 'Reference Pose complete line', 'Vertices complete line', 'Manual teaching' );
title('Welding Trajectory');

saveName = fullfile('.', strcat('results/plot', '.eps'));
eval(['print -painters -depsc ' saveName]);