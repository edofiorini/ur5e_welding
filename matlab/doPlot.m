%%%%% Matlab script for plotting and saving data %%%%%
clear all; close all; clc;

% import data
robot_pointer_csv_name = 'paper_csv/robot_clothoids.csv';
ref_pointer_csv_name = 'paper_csv/ref_clothoids.csv';
vertices_pointer_csv_name = 'paper_csv/vertices_clothoids.csv';


robot_complete_csv_name = 'paper_csv/robot_complete.csv'; 
ref_complete_csv_name = 'paper_csv/ref_complete_mix.csv';
vertices_complete_csv_name = 'paper_csv/vertices_complete_mix.csv';

manual_name = 'paper_csv/robot_manual.csv';
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

manual_x = manual_x - manual_x(1,1);
manual_y = manual_y - manual_y(1,1);

% Extract x, y, and z values
robot_pointer_x = robot_pointer_data(:, 2);
robot_pointer_y = robot_pointer_data(:, 3);
robot_pointer_z = robot_pointer_data(:, 4);

% delete begin
robot_pointer_x = robot_pointer_x(500:end,:);
robot_pointer_y = robot_pointer_y(500:end,:);
robot_pointer_z = robot_pointer_z(500:end,:);

robot_pointer_x = robot_pointer_x - robot_pointer_x(1,1);
robot_pointer_y = robot_pointer_y - robot_pointer_y(1,1);
robot_pointer_z = robot_pointer_z(1,1);

robot_complete_x = robot_complete_data(:, 2);
robot_complete_y = robot_complete_data(:, 3);
robot_rec_z = robot_complete_data(:, 4);

robot_complete_x = robot_complete_x(140:end,:);
robot_complete_y = robot_complete_y(140:end,:);
robot_complete_z = robot_rec_z(140:end,:);

robot_complete_x = robot_complete_x - robot_complete_x(1,1);
robot_complete_y = robot_complete_y - robot_complete_y(1,1);
robot_complete_z = robot_complete_z(1,1);

% Extract x, y, and z values
ref_pointer_x = ref_pointer_data(:, 1);
ref_pointer_y = ref_pointer_data(:, 2);
ref_z = ref_pointer_data(:, 3);

% delete begin
ref_pointer_x = ref_pointer_x(100:end,:);
ref_pointer_y = ref_pointer_y(100:end,:);
ref_z = ref_z(20:end,:);

ref_complete_x = ref_complete_data(:, 1);
ref_complete_y = ref_complete_data(:, 2);
ref_rec_z = ref_complete_data(:, 3);

ref_complete_x = ref_complete_x(20:end,:);
ref_complete_y = ref_complete_y(20:end,:);
ref_complete_z = ref_rec_z(20:end,:);

% Extract x, y, and z values
vert_pointer_x = vert_pointer_data(:, 1);
vert_pointer_y = vert_pointer_data(:, 2);
vert_z = vert_pointer_data(:, 3);

vert_pointer_x = vert_pointer_x - vert_pointer_x(1,1);
vert_pointer_y = vert_pointer_y - vert_pointer_y(1,1);
%vert_pointer_x(3,1) = vert_pointer_x(3,1) + 0.002;
%vert_pointer_x(6,1) = vert_pointer_x(6,1) - 0.002899;

vert_complete_x =  vert_complete_data(:, 1);
vert_complete_y = vert_complete_data(:, 2);
vert_rec_z = vert_complete_data(:, 3);
vert_complete_x(3,1) = vert_complete_x(3,1) + 0.001;
vert_complete_x(6,1) = vert_complete_x(6,1) - 0.003;

vert_complete_x = vert_complete_x - vert_complete_x(1,1);
vert_complete_y = vert_complete_y - vert_complete_y(1,1);

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
hold on;
% pointer method
plot(robot_pointer_x, robot_pointer_y, '.', 'Color', robot_pointer, 'Linewidth', LineWidth_size);
hold on;
%plot(ref_pointer_x, ref_pointer_y, '.', 'Color', ref_pointer, 'Linewidth', LineWidth_size);
hold on;
scatter(vert_pointer_x, vert_pointer_y,'.', 'Color', vert_pointer, 'Linewidth', LineWidth_size);
hold on;
% complete line method
plot(robot_complete_x, robot_complete_y, '.', 'Color', robot_complete, 'Linewidth', LineWidth_size);
hold on;
%plot(ref_complete_x, ref_complete_y, '.', 'Color', ref_complete, 'Linewidth', LineWidth_size);
hold on;

scatter(vert_complete_x, vert_complete_y,'.', 'Color', vert_complete, 'Linewidth', LineWidth_size);
hold on;

% manual 
scatter(manual_x(900:end,:), manual_y(900:end, :), '.', 'Color', 'green', 'Linewidth', LineWidth_size);


axis equal
xlabel('X');
ylabel('Y');
legend('Robot Pose clothoids', 'Vertices clothoids', 'Robot Pose complete line', 'Vertices complete line', 'Manual');
title('Welding Trajectory');

saveName = fullfile('.', strcat('plot', '.eps'));
eval(['print -painters -depsc ' saveName]);